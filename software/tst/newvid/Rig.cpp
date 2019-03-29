#include "Rig.h"

class Counter
{
public:

    int get(guint32 id)
    {
        int ret = 0;
        auto it = mMap.find(id);

        if( mMap.end() == it )
        {
            return it->second;
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

protected:

    std::map<guint32,int> mMap;
};

void Rig::RigProc(Rig* rig)
{
    bool go_on = true;

    const int N_views = static_cast<int>(rig->mCameras.size());

    std::array<ArvBuffer*,GENICAM_NUM_BUFFERS+1> buffers;

    std::vector<ArvBuffer*> chosen_buffers(N_views, nullptr);

    Counter count;

    for(CameraPtr cam : rig->mCameras)
    {
        cam->mTab2.clear();
    }

    while(go_on)
    {
        rig->mSemaphore.down();

        if(rig->mAskThreadToQuit)
        {
            go_on = false;
        }
        else
        {
            bool found = false;
            guint32 found_id = 0;

            for(CameraPtr cam : rig->mCameras)
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

            if(found)
            {
                // put aside buffers corresponding to found_id.

                for(int i=0; i<N_views; i++)
                {
                    if( rig->mCameras[i]->mTab2.find(found_id) == rig->mCameras[i]->mTab2.end() ) throw std::runtime_error("internal error");
                    chosen_buffers[i] = rig->mCameras[i]->mTab2[found_id];
                    rig->mCameras[i]->mTab2.erase(found_id);
                }

                // remove buffers older than found_id and push them to their respective stream.

                for(CameraPtr cam : rig->mCameras)
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

                std::vector<cv::Mat> frames(N_views);

                const double timestamp = static_cast<double>(arv_buffer_get_timestamp(chosen_buffers.front())) * 1.0e-9;

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

                    arv_stream_push_buffer(rig->mCameras[i]->mStream, chosen_buffers[i]);

                    chosen_buffers[i] = nullptr;
                }

                Image image(std::move(frames));
                rig->mMutexB.lock();
                rig->mImage = std::move(image);
                rig->mMutexB.unlock();
                rig->mMutexA.unlock();
                std::cout << "Generated frame " << found_id << std::endl;
            }
            else
            {
                // remove too old buffers.

                for(CameraPtr cam : rig->mCameras)
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

Rig::Rig(const std::initializer_list<int>& cams)
{
    for(int i : cams)
    {
        const std::string id = arv_get_device_id(i);
        mCameras.emplace_back( new Camera(this, id, i) );
    }
}

void Rig::open()
{
    for( CameraPtr cam : mCameras )
    {
        cam->open();
    }

    mIsOpen = true;
    mAskThreadToQuit = false;

    mThread = std::thread(RigProc, this);

    mMutexA.try_lock(); // TODO: if already locked, generate an error.
}

void Rig::close()
{
    mAskThreadToQuit = true;
    mSemaphore.up();
    mThread.join();

    for( CameraPtr cam : mCameras )
    {
        cam->close();
    }

    mIsOpen = false;
}

bool Rig::read(Image& im)
{
    bool ret = false;

    //mMutexA.lock(); // TODO: add timeout wait.
    ret = mMutexA.try_lock_for(std::chrono::milliseconds(100)); // TODO: add timeout wait.

    if(ret)
    {
        mMutexB.lock();
        im = std::move(mImage);
        mMutexB.unlock();
    }

    return ret;
}

void Rig::trigger()
{
    for(CameraPtr cam : mCameras)
    {
        cam->trigger();
    }
}

