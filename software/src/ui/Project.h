#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QDir>
#include <sophus/se3.hpp>
#include "CameraCalibrationModel.h"
#include "RigCalibrationModel.h"
#include "RecordingModel.h"
#include "ReconstructionModel.h"
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "CameraCalibrationList.h"
#include "RigCalibrationList.h"
#include "RecordingList.h"

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

    void clear(); // Reset the project! Use with caution!

    void beginTransaction();
    void endTransaction();
    void abortTransaction();

    bool saveCamera(CameraCalibrationDataPtr camera, int& id);
    bool loadCamera(int id, CameraCalibrationDataPtr& data);
    bool listCameras(CameraCalibrationList& list);
    bool isCameraMutable(int id, bool& mut);

    bool saveRig(StereoRigCalibrationDataPtr rig, int& id);
    bool loadRig(int id, StereoRigCalibrationDataPtr& rig);
    bool listRigs(RigCalibrationList& list);
    bool isRigMutable(int id, bool& mut);

    bool listRecordings(RecordingList& list);

    bool savePose(const Sophus::SE3d& pose, int& id);
    bool loadPose(int id, Sophus::SE3d& pose);

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

