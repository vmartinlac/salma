#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QDir>
#include <sophus/se3.hpp>
#include "CameraCalibrationModel.h"
#include "RigCalibrationModel.h"
#include "RecordingModel.h"
#include "ReconstructionModel.h"

class CameraCalibrationData;
class StereoRigCalibrationData;

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

    void beginTransaction();
    void endTransaction();
    void abortTransaction();

    bool saveCamera(const CameraCalibrationData& camera, int& id);
    bool loadCamera(int id, CameraCalibrationData& data);
    bool listCameras(CameraCalibrationList& list);
    bool isCameraMutable(int id, bool& mut);

    bool saveRig(const StereoRigCalibrationData& rig, int& id);
    bool loadRig(int id, StereoRigCalibrationData& rig);
    bool listRigs(RigCalibrationList& list);
    bool isRigMutable(int id, bool& mut);

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

