#pragma once

#include "Model.h"
#include "ReconstructionList.h"

class ReconstructionModel : public Model
{
    Q_OBJECT

public:

    ReconstructionModel(Project* parent);

    int rowCount(const QModelIndex& parent=QModelIndex()) const override;

    QVariant data(const QModelIndex& index, int role=Qt::DisplayRole) const override;

    int indexToId(const QModelIndex& index) override;

public slots:

    void refresh() override;

protected:

    int convertIndex(const QModelIndex& ind) const;

protected:

    ReconstructionList mReconstructions;
};

