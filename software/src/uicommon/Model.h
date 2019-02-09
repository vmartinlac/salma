#pragma once

#include <QAbstractListModel>

class Project;

class Model : public QAbstractListModel
{
    Q_OBJECT

public:

    Model(Project* parent);

    //int rowCount(const QModelIndex& parent=QModelIndex()) const override;

    //QVariant data(const QModelIndex& index, int role=Qt::DisplayRole) const override;

    virtual int indexToId(const QModelIndex& index) = 0;

    Project* project();

protected slots:

    virtual void refresh() = 0;

private:

    Project* mProject;
};

