#include "CalibrationModel.h"
#include "Project.h"

CalibrationModel::CalibrationModel(Project* parent) : Model(parent)
{
}

int CalibrationModel::rowCount(const QModelIndex& parent) const
{
    int ret = 0;

    if(parent.isValid() == false)
    {
        ret = mRigs.size();
    }

    return ret;
}

QVariant CalibrationModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    int i = convertIndex(index);

    if( i >= 0 && role == Qt::DisplayRole )
    {
        switch( index.column() )
        {
        case 0:
            ret = mRigs[i].name;
            break;
        case 1:
            ret = mRigs[i].date;
            break;
        }
    }

    return ret;
}

void CalibrationModel::refresh()
{
    beginResetModel();
    mRigs.clear();
    project()->listCalibrations(mRigs);
    endResetModel();
}

int CalibrationModel::indexToId(const QModelIndex& index)
{
    const int ind2 = convertIndex(index);

    if(ind2 >= 0)
    {
        return mRigs[ind2].id;
    }
    else
    {
        return -1;
    }
}

int CalibrationModel::convertIndex(const QModelIndex& ind) const
{
    if( ind.isValid() && 0 <= ind.row() && ind.row() < mRigs.size() && ind.parent().isValid() == false )
    {
        return ind.row();
    }
    else
    {
        return -1;
    }
}

