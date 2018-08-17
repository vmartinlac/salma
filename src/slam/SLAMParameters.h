#pragma once

#include <QJsonDocument>
#include <QString>

class SLAMParameters
{
public:

    double cx;
    double cy;
    double fx;
    double fy;
    double distortion_k1;
    double distortion_k2;
    double distortion_k3;
    double distortion_p1;
    double distortion_p2;
    double calibration_target_scale; // the length of the side of a case of the calibration target.
    int patch_size;
    double min_distance_to_camera;
    int max_landmark_candidates;
    int num_depth_hypotheses;
    double min_depth_hypothesis;
    double max_depth_hypothesis;

    SLAMParameters();
    bool loadFromJson(const QJsonDocument& doc);
    bool saveToJson(QJsonDocument& doc);
    bool loadFromFile(const QString& file);
    bool saveToFile(const QString& file);
    bool loadFromSettings();
    bool saveToSettings();
    void reset();
};
