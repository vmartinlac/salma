#pragma once

#include <QDialog>
#include <QPlainTextEdit>
#include "StereoRigCalibration.h"
#include "CalibrationResiduals.h"

class CalibrationAcceptationDialog : public QDialog
{
public:

    CalibrationAcceptationDialog(QWidget* parent=nullptr);

    void setData(
        StereoRigCalibrationPtr calib,
        CalibrationResiduals& residuals);

protected:

    QPlainTextEdit* mText;
};

