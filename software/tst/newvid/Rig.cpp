#include "Rig.h"

void Rig::RigProc(Rig* rig)
{
    bool go_on = true;

    const int N_views = static_cast<int>(rig->mCameras.size());

    std::array<ArvBuffer*,GENICAM_NUM_BUFFERS+1> buffers;

    std::vector<ArvBuffer*> chosen_buffers(N_views, nullptr);

    std::map<guint32,int> count;

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

                    {
                        std::map<guint32,int>::iterator it2 = count.find(id);

                        if( count.end() == it2 )
                        {
                            count[id] = 1;
                        }
                        else
                        {
                            it2->second++;
                        }
                    }

                    if( count[id] == N_views && (found == false || id > found_id))
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

                count.erase(found_id);

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

                {
                    std::map<guint32,int>::iterator it = count.begin();

                    while(it != count.end())
                    {
                        if(it->first <= found_id)
                        {
                            it = count.erase(it);
                        }
                        else
                        {
                            it++;
                        }
                    }
                }

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

                // TODO: put these frame in an image.
                std::cout << "Generated a frame!" << std::endl;
            }
            else
            {
                // TODO: put too old buffers.
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

cv::Mat Rig::read()
{
    return cv::Mat();
}

