#pragma once

#include <QDialog>
#include <QDir>
#include <QLineEdit>
#include <QComboBox>
#include "Camera.h"
#include "Parameters.h"

class ParametersDialog : public QDialog
{
    Q_OBJECT
public:

    ParametersDialog(Parameters* parameters, QWidget* parent=nullptr);

public slots:

    int exec() override;

protected slots:

    void accept();
    void selectOutputDirectory();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;

    Parameters* mParameters;
};

