#pragma once

#include "Model.h"

class RigCalibrationModel : public Model
{
    Q_OBJECT

public:

    RigCalibrationModel(Project* parent);

    int rowCount(const QModelIndex& parent=QModelIndex()) const override;

    QVariant data(const QModelIndex& index, int role=Qt::DisplayRole) const override;

    int indexToId(const QModelIndex& index) override;

protected slots:

    void refresh() override;

protected:

    struct Item
    {
        int rig_id;
        QString name;
        QString date;
    };

protected:

    std::vector<Item> mItems;
};

