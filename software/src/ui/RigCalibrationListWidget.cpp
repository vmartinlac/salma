#include "RigCalibrationListWidget.h"
#include "RigCalibrationModel.h"
#include "Project.h"

RigCalibrationListWidget::RigCalibrationListWidget(Project* project, QWidget* parent) : QComboBox(parent)
{
    mProject = project;

    // TODO: connect model to the combobox with setModel().

    refreshList();
}

RigCalibrationListWidget::~RigCalibrationListWidget()
{
}

void RigCalibrationListWidget::refreshList()
{
    clear();

    RigCalibrationList list;
    const bool ok = mProject->listRigs(list);

    if(ok)
    {
        for(RigCalibrationListItem& item : list)
        {
            addItem(item.name, item.id);
        }
    }
}

int RigCalibrationListWidget::getRigCalibrationId()
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

