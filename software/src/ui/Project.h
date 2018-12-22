#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QDir>
#include "CameraCalibrationModel.h"
#include "RigCalibrationModel.h"
#include "RecordingModel.h"
#include "ReconstructionModel.h"

class Project : public QObject
{
    Q_OBJECT

public:

    Project(QObject* parent=nullptr);

    bool open(const QString& path);
    void close();

    CameraCalibrationModel* cameraCalibrationModel();
    RigCalibrationModel* rigCalibrationModel();
    RecordingModel* recordingModel();
    ReconstructionModel* reconstructionModel();

signals:
    
    void changed();
    void cameraCalibrationModelChanged();
    void rigCalibrationModelChanged();
    void recordingModelChanged();
    void reconstructionModelChanged();

protected:

    QDir mDir;
    QSqlDatabase mDB;
    CameraCalibrationModel* mCameraCalibrationModel;
    RigCalibrationModel* mRigCalibrationModel;
    RecordingModel* mRecordingModel;
    ReconstructionModel* mReconstructionModel;
};

