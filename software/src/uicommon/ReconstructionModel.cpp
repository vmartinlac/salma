#include "ReconstructionModel.h"
#include "Project.h"

ReconstructionModel::ReconstructionModel(Project* parent) : Model(parent)
{
}

int ReconstructionModel::rowCount(const QModelIndex& parent) const
{
    int ret = 0;

    if( parent.isValid() == false )
    {
        ret = mReconstructions.size();
    }

    return ret;
}

QVariant ReconstructionModel::data(const QModelIndex& index, int role) const
{
    QVariant ret;

    int i = convertIndex(index);

    if( i >= 0 && role == Qt::DisplayRole )
    {
        switch( index.column() )
        {
        case 0:
            ret = mReconstructions[i].name;
            break;
        case 1:
            ret = mReconstructions[i].date;
            break;
        }
    }

    return ret;
}

void ReconstructionModel::refresh()
{
    beginResetModel();
    mReconstructions.clear();
    project()->listReconstructions(mReconstructions);
    endResetModel();
}

int ReconstructionModel::indexToId(const QModelIndex& index)
{
    const int ind2 = convertIndex(index);

    if(ind2 >= 0)
    {
        return mReconstructions[ind2].id;
    }
    else
    {
        return -1;
    }
}

int ReconstructionModel::convertIndex(const QModelIndex& ind) const
{
    if( ind.isValid() && 0 <= ind.row() && ind.row() < mReconstructions.size() && ind.parent().isValid() == false )
    {
        return ind.row();
    }
    else
    {
        return -1;
    }
}

