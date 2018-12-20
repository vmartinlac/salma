#pragma once

#include <QTextEdit>
#include <QTreeView>
#include <QWidget>
#include "CameraCalibrationModel.h"

class CameraCalibrationPanel : public QWidget
{
    Q_OBJECT

public:

    CameraCalibrationPanel(QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onRename();
    void onImport();
    void onExport();
    void onDelete();

protected:

    QTreeView* mView;
    CameraCalibrationModel* mModel;
    QTextEdit* mText;
};

