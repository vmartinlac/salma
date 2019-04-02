#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QDir>
#include <sophus/se3.hpp>
#include "RecordingModel.h"
#include "CalibrationModel.h"
#include "ReconstructionModel.h"
#include "StereoRigCalibration.h"
#include "CalibrationList.h"
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

    RecordingModel* recordingModel();
    CalibrationModel* calibrationModel();
    ReconstructionModel* reconstructionModel();

    bool saveRecording(RecordingHeaderPtr rec, int& id);
    bool loadRecording(int id, RecordingHeaderPtr& rec);
    bool listRecordings(RecordingList& list);
    bool isRecordingMutable(int id, bool& mut);
    bool describeRecording(int id, QString& descr);
    bool renameRecording(int id, const QString& new_name);
    bool removeRecording(int id);
    bool createRecordingDirectory(QDir& dir);

    bool saveCalibration(StereoRigCalibrationPtr rig, int& id);
    bool loadCalibration(int id, StereoRigCalibrationPtr& rig);
    bool listCalibrations(CalibrationList& list);
    bool isCalibrationMutable(int id, bool& mut);
    bool describeCalibration(int id, QString& descr);
    bool renameCalibration(int id, const QString& new_name);
    bool removeCalibration(int id);

    bool saveReconstruction(SLAMReconstructionPtr rec, int& id);
    bool loadReconstruction(int id, SLAMReconstructionPtr& rec);
    bool listReconstructions(ReconstructionList& list);
    bool isReconstructionMutable(int id, bool& mut);
    bool describeReconstruction(int id, QString& descr);
    bool renameReconstruction(int id, const QString& new_name);
    bool removeReconstruction(int id);

protected:

    bool savePose(const Sophus::SE3d& pose, int& id);
    bool loadPose(int id, Sophus::SE3d& pose);

    bool saveMapPoint(SLAMMapPointPtr mappoint, int& id);
    bool loadMapPoint(int id, SLAMMapPointPtr& mappoint);

    bool saveFrame(SLAMFramePtr frame, int rank, int reconstruction_id, int& id);

    bool loadKeyPoints(int frame_id, SLAMFramePtr frame);
    bool loadDensePoints(int frame_id, SLAMFramePtr frame);

    bool saveCamera(CameraCalibration& camera, int rig_id, int rank, int& id);
    bool loadCamera(int id, CameraCalibration& camera);

    static std::string htmlEscape(const std::string& from);

signals:
    
    void changed();
    void calibrationModelChanged();
    void recordingModelChanged();
    void reconstructionModelChanged();

protected:

    QDir mDir;
    QSqlDatabase mDB;

    RecordingModel* mRecordingModel;
    CalibrationModel* mCalibrationModel;
    ReconstructionModel* mReconstructionModel;

    std::map<int,int> mMapPointToDB;
    std::map<int,SLAMMapPointPtr> mMapPointFromDB;
};

