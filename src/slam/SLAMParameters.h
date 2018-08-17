#pragma once

#include <QJsonDocument>
#include <QString>
#include <opencv2/core.hpp>

class SLAMParameters
{
public:

    cv::Mat calibration_matrix;
    cv::Mat distortion_coefficients;
    double calibration_target_scale; // the length of the side of a case of the calibration target.
    int patch_size;
    double min_distance_to_camera;

    int num_depth_hypotheses;
    double min_depth_hypothesis;
    double max_depth_hypothesis;

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
