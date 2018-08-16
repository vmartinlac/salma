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
    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    m_fx = new QLineEdit();
    m_fy = new QLineEdit();
    m_cx = new QLineEdit();
    m_cy = new QLineEdit();

    m_fx->setValidator(m_double_validator);
    m_fy->setValidator(m_double_validator);
    m_cx->setValidator(m_double_validator);
    m_cy->setValidator(m_double_validator);

    lay->addRow("fx", m_fx);
    lay->addRow("fy", m_fy);
    lay->addRow("cx", m_cx);
    lay->addRow("cy", m_cy);

    return ret;
}

QWidget* SLAMParametersDialog::create_engine_tab()
{
    QWidget* ret = new QWidget();

    QFormLayout* lay = new QFormLayout();
    ret->setLayout(lay);

    lay->addRow("patch size", new QLineEdit());

    return ret;
}

SLAMParametersDialog::SLAMParametersDialog(QWidget* parent) : QDialog(parent)
{
    m_double_validator = new QDoubleValidator(this);

    QTabWidget* tab = new QTabWidget();
    tab->addTab(create_camera_tab(), "Camera");
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
        const int ok = m_parameters.loadFromFile(ret);
        if(ok == false)
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
        const int ok = m_parameters.saveToFile(ret);
        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Error");
        }
    }
}

bool SLAMParametersDialog::ask(QWidget* parent, SLAMParameters& parameters)
{
    SLAMParametersDialog* dlg = new SLAMParametersDialog(parent);

    dlg->toUserInterface(parameters);

    const int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        dlg->fromUserInterface(parameters);
    }

    delete dlg;

    return (ret == QDialog::Accepted);
}

void SLAMParametersDialog::fromUserInterface(SLAMParameters& parameters)
{
    ;
}

void SLAMParametersDialog::toUserInterface(SLAMParameters& parameters)
{
    ;
}

