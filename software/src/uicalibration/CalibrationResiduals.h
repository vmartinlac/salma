#pragma once

struct CalibrationResiduals
{
    CalibrationResiduals();

    double left_camera_calibration;
    double right_camera_calibration;
    double stereo_calibration;
};

