#include "CameraCalibrationListWidget.h"
#include "CameraCalibrationModel.h"
#include "Project.h"

CameraCalibrationListWidget::CameraCalibrationListWidget(Project* project, QWidget* parent) : QComboBox(parent)
{
    mProject = project;

    // TODO: connect model to the combobox with setModel().

    refreshList();
}

CameraCalibrationListWidget::~CameraCalibrationListWidget()
{
}

void CameraCalibrationListWidget::refreshList()
{
    clear();

    CameraCalibrationList list;
    const bool ok = mProject->listCameras(list);

    if(ok)
    {
        for(CameraCalibrationListItem& item : list)
        {
            addItem(item.name, item.id);
        }
    }
}

int CameraCalibrationListWidget::getCameraCalibrationId()
{
    bool ok;
    int value = currentData().toInt(&ok);

    if(ok)
    {
        return value;
    }
    else
    {
        return -1;
    }
}

