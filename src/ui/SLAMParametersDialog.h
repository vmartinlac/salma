#pragma once

#include <QLineEdit>
#include <QDialog>
#include <QDoubleValidator>
#include "SLAMParameters.h"

class SLAMParametersDialog : public QDialog
{
    Q_OBJECT
public:

    SLAMParametersDialog(QWidget* parent);

    void storeToUI(const SLAMParameters& parameters);
    void storeFromUI(SLAMParameters& parameters);

    static bool ask(QWidget* parent, SLAMParameters& parameters);

protected slots:

    void loadFromFile();
    void saveToFile();

protected:

    QWidget* create_camera_tab();
    QWidget* create_target_tab();
    QWidget* create_engine_tab();

protected:

    QDoubleValidator* m_double_validator;

    QLineEdit* m_fx;
    QLineEdit* m_fy;
    QLineEdit* m_cx;
    QLineEdit* m_cy;
    QLineEdit* m_distortion_k1;
    QLineEdit* m_distortion_k2;
    QLineEdit* m_distortion_k3;
    QLineEdit* m_distortion_p1;
    QLineEdit* m_distortion_p2;
    QLineEdit* m_calibration_target_scale;
    QLineEdit* m_patch_size;
    QLineEdit* m_min_distance_to_camera;
    QLineEdit* m_max_landmark_candidates;
    QLineEdit* m_num_depth_hypotheses;
    QLineEdit* m_min_depth_hypothesis;
    QLineEdit* m_max_depth_hypothesis;
};

