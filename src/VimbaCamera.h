#pragma once

#include "Camera.h"
#include <VimbaC/Include/VimbaC.h>
#include <string>
#include <mutex>
#include <vector>

// VimbaCamera

class VimbaCamera : public Camera
{

public:

    VimbaCamera(int id, const VmbCameraInfo_t& infos);
    ~VimbaCamera() override;

    std::string getHumanName() override;

    bool start() override;

    void stop() override;

    Image* readImage() override;

protected:

    static void VMB_CALL frame_callback( const VmbHandle_t camera, VmbFrame_t* frame );

protected:

    std::string m_camera_id;
    std::string m_camera_name;
    std::string m_camera_model;
    std::string m_camera_serial;
    VmbAccessMode_t m_camera_permitted_access;
    std::string m_interface_id;

    bool m_is_open;
    VmbHandle_t m_handle;
    std::vector<VmbFrame_t> m_frames;
    std::mutex m_mutex;
    Image* m_newest_image;
};

// CameraManager

class VimbaCameraManager : public CameraManager
{

public:

    bool initialize() override;

    void finalize() override;

    Camera* getDefaultCamera() override;

    int getNumCameras() override;

    Camera* getCamera(int id) override;

protected:

    int m_num_cameras;
    std::vector<VimbaCamera*> m_cameras;
};

