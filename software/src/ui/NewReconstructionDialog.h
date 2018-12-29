
#pragma once

#include "NewOperationDialog.h"
#include "CameraList.h"
#include "TargetScaleWidget.h"
#include <QLineEdit>

class NewReconstructionDialog : public NewOperationDialog
{
    Q_OBJECT

public:

    NewReconstructionDialog(Project* proj, QWidget* parent=nullptr);

protected slots:

    void accept() override;

protected:

    QLineEdit* mName;
};

