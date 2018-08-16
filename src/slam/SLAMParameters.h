#pragma once

#include <QJsonDocument>
#include <QString>
#include <opencv2/core.hpp>

class SLAMParameters
{
public:

    cv::Mat calibration_matrix;
    cv::Mat distortion_coefficients;
    double calibration_target_scale;
    int patch_size;
    int num_depth_hypotheses;
    double min_distance_to_camera;
    cv::Rect image_viewport;

    SLAMParameters();
    bool loadFromJson(const QJsonDocument& doc);
    bool saveToJson(const QJsonDocument& doc);
    bool loadFromFile(const QString& file);
    bool saveToFile(const QString& file);
    bool loadFromDefaultParameterFile();
    bool saveToDefaultParameterFile();
    void reset();

protected:

    static QString getDefaultParameterFileName();
};
