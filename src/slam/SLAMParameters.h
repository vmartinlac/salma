#pragma once

#include <QJsonDocument>
#include <QString>

class SLAMParameters
{
public:

    enum InitializationTargetKind
    {
        INITIALIZATION_TARGET_ONE_PLANE=0,
        INITIALIZATION_TARGET_TWO_PLANE=1
    };

    double cx;
    double cy;
    double fx;
    double fy;
    double distortion_k1;
    double distortion_k2;
    double distortion_k3;
    double distortion_p1;
    double distortion_p2;
    double initialization_target_scale;
    int initialization_target_kind;
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
