#pragma once

#include <QAbstractListModel>

class CameraCalibrationModel : public QAbstractListModel
{
    Q_OBJECT

public:

    CameraCalibrationModel(QObject* parent=nullptr);

    int rowCount(const QModelIndex& parent=QModelIndex()) const override;

    QVariant data(const QModelIndex& index, int role=Qt::DisplayRole) const override;

protected:
};

