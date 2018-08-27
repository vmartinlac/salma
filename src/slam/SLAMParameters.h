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

    // camera parameters.
    double cx;
    double cy;
    double fx;
    double fy;
    double distortion_k1;
    double distortion_k2;
    double distortion_k3;
    double distortion_p1;
    double distortion_p2;

    // calibration target parameters.
    double initialization_target_scale; // the length of the side of a case of the initialization target.
    int initialization_target_kind;

    // slam engine parameters.
    int patch_size;
    double min_distance_to_camera;
    int max_landmark_candidates; // TODO: rather say candidate landmarks ?
    int num_depth_hypotheses;
    double min_depth_hypothesis;
    double max_depth_hypothesis;
    int min_init_landmarks;
    double gftt_quality_level;
    int gftt_max_corners;

    // Maximal number of landmarks considered as found on one frame.
    // we limit the number of landmarks so that the linear system
    // to inverse at each frame is not too large and computation
    // time remains low consequently.
    int max_landmarks_per_frame; // TODO: save this parameter from/to json. let the user set it from the ui.

    SLAMParameters();
    bool loadFromJson(const QJsonDocument& doc);
    bool saveToJson(QJsonDocument& doc);
    bool loadFromFile(const QString& file);
    bool saveToFile(const QString& file);
    bool loadFromSettings();
    bool saveToSettings();
    void reset();
};
