#include "CalibrationListWidget.h"
#include "CalibrationModel.h"
#include "Project.h"

CalibrationListWidget::CalibrationListWidget(Project* project, QWidget* parent) : QComboBox(parent)
{
    mProject = project;

    // TODO: connect model to the combobox with setModel().

    refreshList();
}

CalibrationListWidget::~CalibrationListWidget()
{
}

void CalibrationListWidget::refreshList()
{
    clear();

    CalibrationList list;
    const bool ok = mProject->listCalibrations(list);

    if(ok)
    {
        for(CalibrationListItem& item : list)
        {
            addItem(item.name, item.id);
        }
    }
}

int CalibrationListWidget::getCalibrationId()
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

