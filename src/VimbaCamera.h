#pragma once

#include "Camera.h"
#include <VimbaC/Include/VimbaC.h>
#include <string>
#include <vector>

// VimbaCamera

class VimbaCamera : public Camera
{

public:

    VimbaCamera(int id, const VmbCameraInfo_t& infos);
    ~VimbaCamera();

    std::string getHumanName() override;

    bool start() override;

    void stop() override;

protected:

    std::string m_camera_id;
    std::string m_camera_name;
    std::string m_camera_model;
    std::string m_camera_serial;
    VmbAccessMode_t m_camera_permitted_access;
    std::string m_interface_id;

    bool m_is_open;
    VmbHandle_t m_handle;
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

