#include "Project.h"
#include "CameraCalibrationModel.h"

CameraCalibrationModel::CameraCalibrationModel(Project* parent) : Model(parent)
{
}

int CameraCalibrationModel::rowCount(const QModelIndex& parent) const
{
    int ret = 0;

    if(parent.isValid() == false)
    {
        ret = mCameras.size();
    }

    return ret;
}

QVariant CameraCalibrationModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    int i = convertIndex(index);

    if( i >= 0 && role == Qt::DisplayRole )
    {
        switch( index.column() )
        {
        case 0:
            ret = mCameras[i].name;
            break;
        case 1:
            ret = mCameras[i].date;
            break;
        }
    }

    return ret;
}

void CameraCalibrationModel::refresh()
{
    beginResetModel();
    mCameras.clear();
    project()->listCameras(mCameras);
    endResetModel();
}

int CameraCalibrationModel::indexToId(const QModelIndex& index)
{
    const int ind2 = convertIndex(index);

    if(ind2 >= 0)
    {
        return mCameras[ind2].id;
    }
    else
    {
        return -1;
    }
}

int CameraCalibrationModel::convertIndex(const QModelIndex& ind) const
{
    if( ind.isValid() && 0 <= ind.row() && ind.row() < mCameras.size() && ind.parent().isValid() == false )
    {
        return ind.row();
    }
    else
    {
        return -1;
    }
}

