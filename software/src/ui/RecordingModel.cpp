#include "RecordingModel.h"
#include "Project.h"

RecordingModel::RecordingModel(Project* parent) : Model(parent)
{
}

int RecordingModel::rowCount(const QModelIndex& parent) const
{
    if( parent.isValid() == false )
    return 2;
    else return 0;
}

QVariant RecordingModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    if( index.isValid() && index.column() == 0 && role == Qt::DisplayRole )
    {
        if( index.row() == 0)
        {
            ret = "hello";
        }
        else if( index.row() == 1 )
        {
            ret = "world";
        }
    }

    return ret;
}

void RecordingModel::refresh()
{
}

int RecordingModel::indexToId(const QModelIndex& index)
{
	throw; //TODO
}

