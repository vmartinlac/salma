#pragma once

#include <QLineEdit>
#include <QComboBox>
#include <QDialog>
#include <QDoubleValidator>
#include "SLAMParameters.h"

class ParametersDialog : public QDialog
{
    Q_OBJECT
public:

    ParametersDialog(QWidget* parent);

    void storeToUI(const SLAMParameters& parameters);
    bool storeFromUI(SLAMParameters& parameters);

    static bool ask(QWidget* parent, SLAMParameters& parameters);

protected slots:

    void accept() override;
    void loadFromFile();
    void saveToFile();

protected:

    QWidget* create_camera_tab();
    QWidget* create_target_tab();
    QWidget* create_engine_tab();

protected:

    SLAMParameters m_accepted_parameters;

    QLineEdit* m_fx;
    QLineEdit* m_fy;
    QLineEdit* m_cx;
    QLineEdit* m_cy;
    QLineEdit* m_distortion_k1;
    QLineEdit* m_distortion_k2;
    QLineEdit* m_distortion_k3;
    QLineEdit* m_distortion_p1;
    QLineEdit* m_distortion_p2;
    QLineEdit* m_initialization_target_scale;
    QComboBox* m_initialization_target_kind;
    QLineEdit* m_patch_size;
    QLineEdit* m_min_distance_to_camera;
    QLineEdit* m_max_landmark_candidates;
    QLineEdit* m_num_depth_hypotheses;
    QLineEdit* m_min_depth_hypothesis;
    QLineEdit* m_max_depth_hypothesis;
    QLineEdit* m_min_init_landmarks;
    QLineEdit* m_gftt_max_corners;
    QLineEdit* m_gftt_quality_level;
    QLineEdit* m_max_landmarks_per_frame;
};

