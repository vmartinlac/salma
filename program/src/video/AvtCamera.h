#pragma once

#include <memory>
#include <VimbaC/Include/VimbaC.h>
#include "Camera.h"

class AvtCamera : public Camera
{
public:

    enum TriggerMode
    {
        TRIGGER_IO,
        TRIGGER_ETHERNET
    };

    AvtCamera(const VmbCameraInfo_t& caminfos, TriggerMode trigger_mode);
    ~AvtCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;

    void read(Image& image) override;
    void trigger() override;

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

    bool m_is_open;
    VmbHandle_t m_handle;
    VmbInt64_t m_tick_frequency;
    std::vector<Frame> m_frames;
    int m_next_frame;

    int m_max_wait_milliseconds;
    TriggerMode m_trigger_mode;
};

typedef std::shared_ptr<AvtCamera> AvtCameraPtr;

