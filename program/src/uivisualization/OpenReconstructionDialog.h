#pragma once

#include <QDialog>
#include "PathWidget.h"

class OpenReconstructionDialog : public QDialog
{
    Q_OBJECT

public:

    OpenReconstructionDialog(QWidget* p=nullptr);

protected slots:

    void onOK();
    void onCancel();

protected:

    QWidget* createProjectLayout();
    QWidget* createReconstructionLayout();

protected:

    PathWidget* mProjectPath;
};

