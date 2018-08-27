#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "ParametersDialog.h"

QWidget* ParametersDialog::create_camera_tab()
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

QWidget* ParametersDialog::create_target_tab()
{
    m_initialization_target_scale = new QLineEdit();
    m_initialization_target_kind = new QComboBox();

    m_initialization_target_kind->addItem("one plane", SLAMParameters::INITIALIZATION_TARGET_ONE_PLANE);
    m_initialization_target_kind->addItem("two planes", SLAMParameters::INITIALIZATION_TARGET_TWO_PLANE);

    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("initialization target scale", m_initialization_target_scale);
    lay->addRow("initialization target kind", m_initialization_target_kind);

    return ret;
}

QWidget* ParametersDialog::create_engine_tab()
{
    m_patch_size = new QLineEdit();
    m_min_distance_to_camera = new QLineEdit();
    m_max_landmark_candidates = new QLineEdit();
    m_num_depth_hypotheses = new QLineEdit();
    m_min_depth_hypothesis = new QLineEdit();
    m_max_depth_hypothesis = new QLineEdit();
    m_gftt_max_corners = new QLineEdit();
    m_gftt_quality_level = new QLineEdit();
    m_min_init_landmarks = new QLineEdit();
    m_max_landmarks_per_frame = new QLineEdit();

    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("patch size", m_patch_size);
    lay->addRow("min distance to camera", m_min_distance_to_camera);
    lay->addRow("max landmark candidates", m_max_landmark_candidates);
    lay->addRow("num depth hypotheses", m_num_depth_hypotheses);
    lay->addRow("min depth hypothesis", m_min_depth_hypothesis);
    lay->addRow("max depth hypothesis", m_max_depth_hypothesis);
    lay->addRow("GFTT max corners", m_gftt_max_corners);
    lay->addRow("GFTT quality level", m_gftt_quality_level);
    lay->addRow("min init landmarks", m_min_init_landmarks);
    lay->addRow("max landmarks per frame", m_max_landmarks_per_frame);

    return ret;
}

ParametersDialog::ParametersDialog(QWidget* parent) : QDialog(parent)
{
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

void ParametersDialog::loadFromFile()
{
    QString ret = QFileDialog::getOpenFileName(this, "Open", QString(), "JSON file (*.json)");

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

void ParametersDialog::saveToFile()
{
    QString ret = QFileDialog::getSaveFileName(this, "Save", QString(), "JSON file (*.json)");

    if(ret.isEmpty() == false)
    {
        SLAMParameters parameters;

        const bool ok1 = storeFromUI(parameters);

        if(ok1)
        {
            const bool ok2 = parameters.saveToFile(ret);

            if(ok2 == false)
            {
                QMessageBox::critical(this, "Error", "Could not save parameters to file.");
            }
        }
        else
        {
            QMessageBox::critical(this, "Error", "Some field could not be parsed!");
        }
    }
}

void ParametersDialog::accept()
{
    if( storeFromUI(m_accepted_parameters) )
    {
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", "Some field could not be parsed!");
    }
}

bool ParametersDialog::ask(QWidget* parent, SLAMParameters& parameters)
{
    ParametersDialog* dlg = new ParametersDialog(parent);

    dlg->storeToUI(parameters);

    const int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        //dlg->storeFromUI(parameters);
        parameters = dlg->m_accepted_parameters;
    }

    delete dlg;

    return (ret == QDialog::Accepted);
}

void ParametersDialog::storeToUI(const SLAMParameters& parameters)
{
    const int precision = 15;

    m_cx->setText( QString::number(parameters.cx, 'g', precision) );
    m_cy->setText( QString::number(parameters.cy, 'g', precision) );
    m_fx->setText( QString::number(parameters.fx, 'g', precision) );
    m_fy->setText( QString::number(parameters.fy, 'g', precision) );
    m_distortion_k1->setText( QString::number(parameters.distortion_k1, 'g', precision) );
    m_distortion_k2->setText( QString::number(parameters.distortion_k2, 'g', precision) );
    m_distortion_k3->setText( QString::number(parameters.distortion_k3, 'g', precision) );
    m_distortion_p1->setText( QString::number(parameters.distortion_p1, 'g', precision) );
    m_distortion_p2->setText( QString::number(parameters.distortion_p2, 'g', precision) );
    m_initialization_target_scale->setText( QString::number(parameters.initialization_target_scale, 'g', precision) );
    m_initialization_target_kind->setCurrentIndex( std::max(0, m_initialization_target_kind->findData(parameters.initialization_target_kind) ) );
    m_patch_size->setText( QString::number(parameters.patch_size) );
    m_min_distance_to_camera->setText( QString::number(parameters.min_distance_to_camera, 'g', precision) );
    m_max_landmark_candidates->setText( QString::number(parameters.max_landmark_candidates) );
    m_num_depth_hypotheses->setText( QString::number(parameters.num_depth_hypotheses) );
    m_min_depth_hypothesis->setText( QString::number(parameters.min_depth_hypothesis, 'g', precision) );
    m_max_depth_hypothesis->setText( QString::number(parameters.max_depth_hypothesis, 'g', precision) );
    m_gftt_max_corners->setText( QString::number(parameters.gftt_max_corners) );
    m_gftt_quality_level->setText( QString::number(parameters.gftt_quality_level, 'g', precision) );
    m_min_init_landmarks->setText( QString::number(parameters.min_init_landmarks) );
    m_max_landmarks_per_frame->setText( QString::number(parameters.max_landmarks_per_frame) );
}

bool ParametersDialog::storeFromUI(SLAMParameters& parameters)
{
    bool ok = true;
    bool ret = true;

    parameters.cx = m_cx->text().toDouble(&ok); ret = (ret && ok);
    parameters.cy = m_cy->text().toDouble(&ok); ret = (ret && ok);
    parameters.fx = m_fx->text().toDouble(&ok); ret = (ret && ok);
    parameters.fy = m_fy->text().toDouble(&ok); ret = (ret && ok);
    parameters.distortion_k1 = m_distortion_k1->text().toDouble(&ok); ret = (ret && ok);
    parameters.distortion_k2 = m_distortion_k2->text().toDouble(&ok); ret = (ret && ok);
    parameters.distortion_k3 = m_distortion_k3->text().toDouble(&ok); ret = (ret && ok);
    parameters.distortion_p1 = m_distortion_p1->text().toDouble(&ok); ret = (ret && ok);
    parameters.distortion_p2 = m_distortion_p2->text().toDouble(&ok); ret = (ret && ok);
    parameters.initialization_target_scale = m_initialization_target_scale->text().toDouble(&ok); ret = (ret && ok);

    QVariant tmp = m_initialization_target_kind->currentData();
    if(tmp.isValid())
    {
        parameters.initialization_target_kind = tmp.toInt(&ok); ret = (ret && ok);
    }
    else
    {
        parameters.initialization_target_kind = SLAMParameters::INITIALIZATION_TARGET_ONE_PLANE;
    }

    parameters.patch_size = m_patch_size->text().toInt(&ok); ret = (ret && ok);
    parameters.min_distance_to_camera = m_min_distance_to_camera->text().toDouble(&ok); ret = (ret && ok);
    parameters.max_landmark_candidates = m_max_landmark_candidates->text().toInt(&ok); ret = (ret && ok);
    parameters.num_depth_hypotheses = m_num_depth_hypotheses->text().toInt(&ok); ret = (ret && ok);
    parameters.min_depth_hypothesis = m_min_depth_hypothesis->text().toDouble(&ok); ret = (ret && ok);
    parameters.max_depth_hypothesis = m_max_depth_hypothesis->text().toDouble(&ok); ret = (ret && ok);
    parameters.gftt_quality_level = m_gftt_quality_level->text().toDouble(&ok); ret = (ret && ok);
    parameters.gftt_max_corners = m_gftt_max_corners->text().toInt(&ok); ret = (ret && ok);
    parameters.min_init_landmarks = m_min_init_landmarks->text().toInt(&ok); ret = (ret && ok);
    parameters.max_landmarks_per_frame = m_max_landmarks_per_frame->text().toInt(&ok); ret = (ret && ok);

    return ret;
}

