#pragma once

#include <memory>
#include <mutex>
#include <VimbaC/Include/VimbaC.h>
#include "Camera.h"

class AvtCamera : public Camera
{
public:

    AvtCamera(const VmbCameraInfo_t& caminfos);
    ~AvtCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;

    void read(Image& image) override;
    void trigger() override;

protected:

    static void VMB_CALL callback( const VmbHandle_t camera, VmbFrame_t* frame );

protected:

    std::string m_camera_id;
    std::string m_camera_name;
    std::string m_camera_model;
    std::string m_camera_serial;
    VmbAccessMode_t m_camera_permitted_access;
    std::string m_interface_id;

    struct Frame
    {
        VmbFrame_t vimba_frame;
        std::vector<uint8_t> buffer;
    };

    bool mIsOpen;
    VmbHandle_t mHandle;
    VmbInt64_t m_tick_frequency;
    std::vector<Frame> mFrames;

    std::mutex mMutex;
    Image mNewImage;
};

typedef std::shared_ptr<AvtCamera> AvtCameraPtr;

