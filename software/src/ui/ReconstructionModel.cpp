#include "ReconstructionModel.h"
#include "Project.h"

ReconstructionModel::ReconstructionModel(Project* parent) : Model(parent)
{
}

int ReconstructionModel::rowCount(const QModelIndex& parent) const
{
    if( parent.isValid() == false )
    return 2;
    else return 0;
}

QVariant ReconstructionModel::data(const QModelIndex& index, int role) const
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

void ReconstructionModel::refresh()
{
}

int ReconstructionModel::indexToId(const QModelIndex& index)
{
}

