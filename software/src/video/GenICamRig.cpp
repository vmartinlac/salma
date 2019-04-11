#include <iostream>
#include <set>
#include <thread>
#include "GenICamRig.h"
#include "GenICamConfig.h"
#include "ArduinoTrigger.h"

class GenICamRig::Counter
{
public:

    int get(guint32 id)
    {
        int ret = 0;
        const auto it = mMap.find(id);

        if( mMap.end() != it )
        {
            ret = it->second;
        }

        return ret;
    }

    void increment(guint32 id)
    {
        auto it = mMap.find(id);
        if( mMap.end() == it )
        {
            mMap[id] = 1;
        }
        else
        {
            it->second++;
        }
    }

    void decrement(guint32 id)
    {
        auto it = mMap.find(id);
        if( mMap.end() == it )
        {
            throw std::runtime_error("internal error");
        }
        else if(it->second > 1)
        {
            it->second--;
        }
        else
        {
            mMap.erase(it);
        }
    }

    void remove(guint32 id)
    {
        mMap.erase(id);
    }

    void removeUpTo(guint32 id)
    {
        auto it = mMap.begin();
        while(it != mMap.end())
        {
            if(it->first <= id)
            {
                it = mMap.erase(it);
            }
            else
            {
                it++;
            }
        }
    }

    void dump()
    {
        std::cout << "{" << std::endl;
        for( const std::pair<guint32,int> p : mMap )
        {
            std::cout << "[ " << p.first << " ] " << p.second << std::endl;
        }
        std::cout << "}" << std::endl;
    }

protected:

    std::map<guint32,int> mMap;
};

GenICamRig::GenICamRig(const std::initializer_list<std::string>& cameras)
{
    mHasFirstTimestamp = false;
    mFirstTimestamp = 0.0;
    mIsOpen = false;
    setCameras(cameras);
}

void GenICamRig::setCameras(const std::initializer_list<std::string>& cameras)
{
    // check that the rig is not currently open.

    if(mIsOpen) throw std::runtime_error("can not change list of cameras while rig is open");

    // check that camera are different.

    {
        std::set<std::string> set(cameras.begin(), cameras.end());
        if( set.size() != cameras.size() ) throw std::runtime_error("A camera can appear at most once in a rig.");
    }

    // apply changes.

    mCameras.clear();

    int rank = 0;
    for(const std::string& id : cameras)
    {
        GenICamCameraPtr cam( new GenICamCamera(this, id, rank) );
        mCameras.push_back(cam);
        rank++;
    }
}

GenICamRig::~GenICamRig()
{
    if(mIsOpen)
    {
        close();
    }
}


std::string GenICamRig::getHumanName()
{
    std::string ret;
    bool first = true;

    ret = "{ ";

    for(GenICamCameraPtr c : mCameras)
    {
        if(first)
        {
            first = false;
        }
        else
        {
            ret += " ; ";
        }

        ret += c->getId();
    }

    ret += " }";

    return ret;
}

bool GenICamRig::open()
{
    bool ok = true;

    if(mIsOpen) throw std::runtime_error("camera is already open");

    mHasFirstTimestamp = false;
    mFirstTimestamp = 0.0;

    const bool software_trigger = !bool(mTrigger);

    if(ok && bool(mTrigger))
    {
        ok = mTrigger->open();
    }

    for(size_t i = 0; ok && i<mCameras.size(); i++)
    {
        ok = mCameras[i]->open(software_trigger);
    }

    if(ok)
    {
        mAskThreadToQuit = false;

        mThread = std::thread([this] () { produceImages(); });

        const bool locked = mMutexA.try_lock();
        if(locked == false) throw std::runtime_error("internal error");
    }

    mIsOpen = ok;

    return ok;
}

void GenICamRig::close()
{
    if(mIsOpen)
    {

        mAskThreadToQuit = true;
        mSemaphore.up();
        mThread.join();

        for( GenICamCameraPtr cam : mCameras )
        {
            cam->close();
        }

        if(mTrigger)
        {
            mTrigger->close();
        }

        mImage.setInvalid();
        mIsOpen = false;
    }
}

void GenICamRig::trigger()
{
    if(mTrigger)
    {
        mTrigger->trigger();
    }
    else
    {
        for(GenICamCameraPtr c : mCameras)
        {
          c->softwareTrigger();
        }
    }
}

void GenICamRig::read(Image& im)
{
    im.setInvalid();

    const bool locked = mMutexA.try_lock_for(std::chrono::milliseconds(200));

    if(locked)
    {
        mMutexB.lock();
        im = std::move(mImage);
        mMutexB.unlock();
    }
}

int GenICamRig::getNumberOfCameras()
{
    return static_cast<int>(mCameras.size());
}

void GenICamRig::produceImages()
{
    bool go_on = true;

    const int N_views = static_cast<int>(mCameras.size());

    std::array<ArvBuffer*,GENICAM_NUM_BUFFERS+1> buffers;

    std::vector<ArvBuffer*> chosen_buffers(N_views, nullptr);

    Counter count;

    for(GenICamCameraPtr cam : mCameras)
    {
        cam->mTab2.clear();
    }

    while(go_on)
    {
        mSemaphore.down();

        if(mAskThreadToQuit)
        {
            go_on = false;
        }
        else
        {
            bool found = false;
            guint32 found_id = 0;

            for(GenICamCameraPtr cam : mCameras)
            {
                std::fill(buffers.begin(), buffers.end(), nullptr);

                cam->mTab1.take(buffers.begin());

                for(auto it = buffers.begin(); *it != nullptr; it++)
                {
                    const guint32 id = arv_buffer_get_frame_id(*it);

                    if( cam->mTab2.find(id) != cam->mTab2.end() ) std::cerr << "Internal error: frame id already in table!" << std::endl;

                    cam->mTab2[id] = *it;

                    count.increment(id);

                    if( count.get(id) == N_views && (found == false || id > found_id))
                    {
                        found = true;
                        found_id = id;
                    }
                }
            }


            /*
            {
                std::cout << "============" << std::endl;
                count.dump();
                int i = 0;
                for(GenICamCameraPtr c : mCameras)
                {
                    std::cout << "cam #" << i << ": " << c->mTab2.size() << std::endl;
                    i++;
                }
                std::cout << "============" << std::endl;
            }
            */

            //std::cout << found << std::endl;
            if(found)
            {
                // put aside buffers corresponding to found_id.

                for(int i=0; i<N_views; i++)
                {
                    if( mCameras[i]->mTab2.find(found_id) == mCameras[i]->mTab2.end() ) throw std::runtime_error("internal error");
                    chosen_buffers[i] = mCameras[i]->mTab2[found_id];
                    mCameras[i]->mTab2.erase(found_id);
                }

                // remove buffers older than found_id and push them to their respective stream.

                for(GenICamCameraPtr cam : mCameras)
                {
                    std::map<guint32,ArvBuffer*>::iterator it = cam->mTab2.begin();

                    while(it != cam->mTab2.end())
                    {
                        if(it->first <= found_id)
                        {
                            arv_stream_push_buffer(cam->mStream, it->second);
                            it = cam->mTab2.erase(it);
                        }
                        else
                        {
                            it++;
                        }
                    }
                }

                count.removeUpTo(found_id);

                // produce the image.

                std::vector<cv::Mat> frames;

                const double this_timestamp = static_cast<double>(arv_buffer_get_timestamp(chosen_buffers.front())) * 1.0e-9;

                if(mHasFirstTimestamp == false)
                {
                    mHasFirstTimestamp = true;
                    mFirstTimestamp = this_timestamp;
                }

                const double timestamp = this_timestamp - mFirstTimestamp;

                for(int i=0; i<N_views; i++)
                {
                    ArvBuffer* buff = chosen_buffers[i];

                    const gint width = arv_buffer_get_image_width(buff);
                    const gint height = arv_buffer_get_image_height(buff);

                    size_t size;
                    const uint8_t* data = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buff, &size));

                    if(size != width*height*3) throw std::runtime_error("internal error");

                    cv::Mat mat( cv::Size(width, height), CV_8UC3 );
                    std::copy( data, data+size, mat.data );

                    frames.emplace_back(std::move(mat));

                    arv_stream_push_buffer(mCameras[i]->mStream, chosen_buffers[i]);

                    chosen_buffers[i] = nullptr;
                }

                if(frames.size() != N_views) throw std::runtime_error("internal error");

                Image image;
                image.setValid(timestamp, frames);

                mMutexB.lock();
                mImage = std::move(image);
                mMutexB.unlock();
                mMutexA.unlock();

                //std::cout << "Generated frame " << found_id << std::endl;
            }
            else
            {
                // remove too old buffers.

                for(GenICamCameraPtr cam : mCameras)
                {
                    auto it = cam->mTab2.begin();

                    while( cam->mTab2.size() > GENICAM_NUM_BUFFERS-4 )
                    {
                        count.decrement(it->first);
                        arv_stream_push_buffer(cam->mStream, it->second);
                        it = cam->mTab2.erase(it);
                    }
                }
            }
        }
    }
}

void GenICamRig::signalImageAvailability()
{
    mSemaphore.up();
}

void GenICamRig::setSoftwareTrigger()
{
    mTrigger.reset();
}

void GenICamRig::setHardwareTrigger(const std::string& device)
{
    ArduinoTriggerPtr trigger(new ArduinoTrigger());
    trigger->setPathToSerialPort(device);
    mTrigger = trigger;
}

