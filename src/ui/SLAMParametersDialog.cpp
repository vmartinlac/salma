#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "SLAMParametersDialog.h"

QWidget* SLAMParametersDialog::create_camera_tab()
{
    m_fx = new QLineEdit();
    m_fy = new QLineEdit();
    m_cx = new QLineEdit();
    m_cy = new QLineEdit();
    m_distortion_k1 = new QLineEdit();
    m_distortion_k2 = new QLineEdit();
    m_distortion_k3 = new QLineEdit();
    m_distortion_p1 = new QLineEdit();
    m_distortion_p2 = new QLineEdit();

    m_fx->setValidator(m_double_validator);
    m_fy->setValidator(m_double_validator);
    m_cx->setValidator(m_double_validator);
    m_cy->setValidator(m_double_validator);
    m_distortion_k1->setValidator(m_double_validator);
    m_distortion_k2->setValidator(m_double_validator);
    m_distortion_k3->setValidator(m_double_validator);
    m_distortion_p1->setValidator(m_double_validator);
    m_distortion_p2->setValidator(m_double_validator);

    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("fx", m_fx);
    lay->addRow("fy", m_fy);
    lay->addRow("cx", m_cx);
    lay->addRow("cy", m_cy);
    lay->addRow("k1", m_distortion_k1);
    lay->addRow("k2", m_distortion_k2);
    lay->addRow("k3", m_distortion_k3);
    lay->addRow("p1", m_distortion_p1);
    lay->addRow("p2", m_distortion_p2);

    return ret;
}

QWidget* SLAMParametersDialog::create_target_tab()
{
    m_calibration_target_scale = new QLineEdit();

    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("calibration target unit length", m_calibration_target_scale);

    return ret;
}

QWidget* SLAMParametersDialog::create_engine_tab()
{
    m_patch_size = new QLineEdit();
    m_min_distance_to_camera = new QLineEdit();
    m_max_landmark_candidates = new QLineEdit();
    m_num_depth_hypotheses = new QLineEdit();
    m_min_depth_hypothesis = new QLineEdit();
    m_max_depth_hypothesis = new QLineEdit();

    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("patch size", m_patch_size);
    lay->addRow("min distance to camera", m_min_distance_to_camera);
    lay->addRow("max landmark candidates", m_max_landmark_candidates);
    lay->addRow("num depth hypotheses", m_num_depth_hypotheses);
    lay->addRow("min depth hypothesis", m_min_depth_hypothesis);
    lay->addRow("max depth hypothesis", m_max_depth_hypothesis);

    return ret;
}

SLAMParametersDialog::SLAMParametersDialog(QWidget* parent) : QDialog(parent)
{
    m_double_validator = new QDoubleValidator(this);

    QTabWidget* tab = new QTabWidget();
    tab->addTab(create_camera_tab(), "Camera");
    tab->addTab(create_target_tab(), "Target");
    tab->addTab(create_engine_tab(), "Engine");

    QPushButton* btn_ok = new QPushButton("OK");
    QPushButton* btn_cancel = new QPushButton("Cancel");
    QPushButton* btn_load = new QPushButton("Load...");
    QPushButton* btn_save = new QPushButton("Save...");

    QHBoxLayout* btnl = new QHBoxLayout();
    btnl->addWidget(btn_ok);
    btnl->addWidget(btn_cancel);
    btnl->addWidget(btn_load);
    btnl->addWidget(btn_save);

    QVBoxLayout* l = new QVBoxLayout();
    l->addWidget(tab);
    l->addLayout(btnl);

    setLayout(l);
    setWindowTitle("SLAM Parameters");

    connect( btn_ok, SIGNAL(clicked()), this, SLOT(accept()) );
    connect( btn_cancel, SIGNAL(clicked()), this, SLOT(reject()) );
    connect( btn_load, SIGNAL(clicked()), this, SLOT(loadFromFile()) );
    connect( btn_save, SIGNAL(clicked()), this, SLOT(saveToFile()) );
}

void SLAMParametersDialog::loadFromFile()
{
    QString ret = QFileDialog::getOpenFileName(this, "Open");

    if(ret.isEmpty() == false)
    {
        SLAMParameters parameters;

        const int ok = parameters.loadFromFile(ret);

        if(ok)
        {
            storeToUI(parameters);
        }
        else
        {
            QMessageBox::critical(this, "Error", "Error");
        }
    }
}

void SLAMParametersDialog::saveToFile()
{
    QString ret = QFileDialog::getSaveFileName(this, "Save");

    if(ret.isEmpty() == false)
    {
        SLAMParameters parameters;

        storeFromUI(parameters);

        const int ok = parameters.saveToFile(ret);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not save parameters to file.");
        }
    }
}

bool SLAMParametersDialog::ask(QWidget* parent, SLAMParameters& parameters)
{
    SLAMParametersDialog* dlg = new SLAMParametersDialog(parent);

    dlg->storeToUI(parameters);

    const int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        dlg->storeFromUI(parameters);
    }

    delete dlg;

    return (ret == QDialog::Accepted);
}

void SLAMParametersDialog::storeToUI(const SLAMParameters& parameters)
{
    m_cx->setText( QString::number(parameters.cx) );
    m_cy->setText( QString::number(parameters.cy) );
    m_fx->setText( QString::number(parameters.fx) );
    m_fy->setText( QString::number(parameters.fy) );
    m_distortion_k1->setText( QString::number(parameters.distortion_k1) );
    m_distortion_k2->setText( QString::number(parameters.distortion_k2) );
    m_distortion_k3->setText( QString::number(parameters.distortion_k3) );
    m_distortion_p1->setText( QString::number(parameters.distortion_p1) );
    m_distortion_p2->setText( QString::number(parameters.distortion_p2) );
    m_calibration_target_scale->setText( QString::number(parameters.calibration_target_scale) );
    m_patch_size->setText( QString::number(parameters.patch_size) );
    m_min_distance_to_camera->setText( QString::number(parameters.min_distance_to_camera) );
    m_max_landmark_candidates->setText( QString::number(parameters.max_landmark_candidates) );
    m_num_depth_hypotheses->setText( QString::number(parameters.num_depth_hypotheses) );
    m_min_depth_hypothesis->setText( QString::number(parameters.min_depth_hypothesis) );
    m_max_depth_hypothesis->setText( QString::number(parameters.max_depth_hypothesis) );
}

void SLAMParametersDialog::storeFromUI(SLAMParameters& parameters)
{
    parameters.cx = m_cx->text().toDouble();
    parameters.cy = m_cy->text().toDouble();
    parameters.fx = m_fx->text().toDouble();
    parameters.fy = m_fy->text().toDouble();
    parameters.distortion_k1 = m_distortion_k1->text().toDouble();
    parameters.distortion_k2 = m_distortion_k2->text().toDouble();
    parameters.distortion_k3 = m_distortion_k3->text().toDouble();
    parameters.distortion_p1 = m_distortion_p1->text().toDouble();
    parameters.distortion_p2 = m_distortion_p2->text().toDouble();
    parameters.calibration_target_scale = m_calibration_target_scale->text().toDouble(); // the length of the side of a case of the calibration target.
    parameters.patch_size = m_patch_size->text().toInt();
    parameters.min_distance_to_camera = m_min_distance_to_camera->text().toDouble();
    parameters.max_landmark_candidates = m_max_landmark_candidates->text().toInt();
    parameters.num_depth_hypotheses = m_num_depth_hypotheses->text().toInt();
    parameters.min_depth_hypothesis = m_min_depth_hypothesis->text().toDouble();
    parameters.max_depth_hypothesis = m_max_depth_hypothesis->text().toDouble();
}

