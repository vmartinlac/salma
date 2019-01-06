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
#include "RecordingHeader.h"
#include "SLAMDataStructures.h"

class Project : public QObject
{
    Q_OBJECT

public:

    Project(QObject* parent=nullptr);

    bool create(const QString& path);
    bool open(const QString& path);
    bool clear(); // Reset the project! Use with caution!
    void close();
    bool isOpen();

    CameraCalibrationModel* cameraCalibrationModel();
    RigCalibrationModel* rigCalibrationModel();
    RecordingModel* recordingModel();
    ReconstructionModel* reconstructionModel();

    bool transaction();
    bool commit();
    bool rollback();

    bool saveCamera(CameraCalibrationDataPtr camera, int& id);
    bool loadCamera(int id, CameraCalibrationDataPtr& data);
    bool listCameras(CameraCalibrationList& list);
    bool isCameraMutable(int id, bool& mut);
    bool describeCamera(int id, QString& descr);
    bool renameCamera(int id, const QString& new_name);

    bool saveRig(StereoRigCalibrationDataPtr rig, int& id);
    bool loadRig(int id, StereoRigCalibrationDataPtr& rig);
    bool listRigs(RigCalibrationList& list);
    bool isRigMutable(int id, bool& mut);
    bool describeRig(int id, QString& descr);
    bool renameRig(int id, const QString& new_name);

    bool saveRecording(RecordingHeaderPtr rec, int& id);
    bool loadRecording(int id, RecordingHeaderPtr& rec);
    bool listRecordings(RecordingList& list);
    bool isRecordingMutable(int id, bool& mut);
    bool describeRecording(int id, QString& descr);
    bool renameRecording(int id, const QString& new_name);
    bool createRecordingDirectory(QDir& dir);

    bool saveReconstruction(SLAMReconstructionPtr rec, int& id);
    bool loadReconstruction(int id, SLAMReconstructionPtr& rec);
    bool listReconstructions(ReconstructionList& list);
    bool isReconstructionMutable(int id, bool& mut);
    bool describeReconstruction(int id, QString& descr);
    bool renameReconstruction(int id, const QString& new_name);

protected:

    bool savePose(const Sophus::SE3d& pose, int& id);
    bool loadPose(int id, Sophus::SE3d& pose);

    bool saveMapPoint(SLAMMapPointPtr mappoint, int& id);
    bool loadMapPoint(int id, SLAMMapPointPtr& mappoint);

    bool saveFrame(SLAMFramePtr frame, int rank, int reconstruction_id, int& id);

    bool loadKeyPoints(int frame_id, SLAMFramePtr frame);

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

    std::map<int,int> mMapPointToDB;
    std::map<int,SLAMMapPointPtr> mMapPointFromDB;
};

