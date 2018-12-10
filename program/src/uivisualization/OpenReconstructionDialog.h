#pragma once

#include <QDialog>
#include <QListWidget>
#include <memory>
#include "SLAMReconstructionDB.h"
#include "PathWidget.h"

class OpenReconstructionDialog : public QDialog
{
    Q_OBJECT

public:

    OpenReconstructionDialog(QWidget* p=nullptr);

    std::shared_ptr<FrameList> getReconstruction();

protected slots:

    void openDatabase();
    void accept() override;

protected:

    std::shared_ptr<SLAMReconstructionDB> mDB;
    PathWidget* mProjectPath;
    QListWidget* mReconstructionList;
    std::shared_ptr<FrameList> mFrames;
};

