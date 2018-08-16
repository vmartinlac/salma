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

    void toUserInterface(SLAMParameters& parameters);
    void fromUserInterface(SLAMParameters& parameters);

    static bool ask(QWidget* parent, SLAMParameters& parameters);

protected slots:

    void loadFromFile();
    void saveToFile();

protected:

    QWidget* create_camera_tab();
    QWidget* create_engine_tab();

protected:

    SLAMParameters m_parameters;
    QDoubleValidator* m_double_validator;
    QLineEdit* m_fx;
    QLineEdit* m_fy;
    QLineEdit* m_cx;
    QLineEdit* m_cy;
};

