#include "Project.h"
#include "CameraCalibrationModel.h"

CameraCalibrationModel::CameraCalibrationModel(Project* parent) : Model(parent)
{
}

int CameraCalibrationModel::rowCount(const QModelIndex& parent) const
{
    if(parent.isValid() == false)
    {
        return mCameras.size();
    }
    else
    {
        return 0;
    }
}

QVariant CameraCalibrationModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    std::cout << role << " " << Qt::DisplayRole << std::endl;
    int i = convertIndex(index);

    if( i >= 0 && index.column() == 0 )
    {
        ret = mCameras[i].name;
        std::cout << mCameras[i].name.toStdString() << std::endl;
    }
    else if( i >= 0 && index.column() == 1 )
    {
        ret = mCameras[i].date;
    }

    return "hello";
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

