#include "RecordingListWidget.h"
#include "RecordingModel.h"
#include "Project.h"

RecordingListWidget::RecordingListWidget(Project* project, QWidget* parent) : QComboBox(parent)
{
    mProject = project;

    // TODO: connect model to the combobox with setModel().

    refreshList();
}

RecordingListWidget::~RecordingListWidget()
{
}

void RecordingListWidget::refreshList()
{
    clear();

    RecordingList list;
    const bool ok = mProject->listRecordings(list);

    if(ok)
    {
        for(RecordingListItem& item : list)
        {
            addItem(item.name, item.id);
        }
    }
}

int RecordingListWidget::getRecordingId()
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

