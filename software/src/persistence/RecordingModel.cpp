#include "Project.h"
#include "RecordingModel.h"

RecordingModel::RecordingModel(Project* parent) : Model(parent)
{
}

int RecordingModel::rowCount(const QModelIndex& parent) const
{
    int ret = 0;

    if( parent.isValid() == false )
    {
        ret = mRecordings.size();
    }

    return ret;
}

QVariant RecordingModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    int i = convertIndex(index);

    if( i >= 0 && role == Qt::DisplayRole )
    {
        switch( index.column() )
        {
        case 0:
            ret = mRecordings[i].name;
            break;
        case 1:
            ret = mRecordings[i].date;
            break;
        }
    }

    return ret;
}

void RecordingModel::refresh()
{
    beginResetModel();
    mRecordings.clear();
    project()->listRecordings(mRecordings);
    endResetModel();
}

int RecordingModel::indexToId(const QModelIndex& index)
{
    const int ind2 = convertIndex(index);

    if(ind2 >= 0)
    {
        return mRecordings[ind2].id;
    }
    else
    {
        return -1;
    }
}

int RecordingModel::convertIndex(const QModelIndex& ind) const
{
    if( ind.isValid() && 0 <= ind.row() && ind.row() < mRecordings.size() && ind.parent().isValid() == false )
    {
        return ind.row();
    }
    else
    {
        return -1;
    }
}

