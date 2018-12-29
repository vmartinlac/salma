#include <QSqlQuery>
#include <QFileInfo>
#include "Project.h"
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "CameraCalibrationList.h"
#include "RigCalibrationList.h"

Project::Project(QObject* parent) : QObject(parent)
{
    mCameraCalibrationModel = new CameraCalibrationModel(this);
    mRigCalibrationModel = new RigCalibrationModel(this);
    mRecordingModel = new RecordingModel(this);
    mReconstructionModel = new ReconstructionModel(this);

    connect(this, SIGNAL(cameraCalibrationModelChanged()), mCameraCalibrationModel, SLOT(refresh()));
    connect(this, SIGNAL(rigCalibrationModelChanged()), mRigCalibrationModel, SLOT(refresh()));
    connect(this, SIGNAL(recordingModelChanged()), mRecordingModel, SLOT(refresh()));
    connect(this, SIGNAL(reconstructionModelChanged()), mReconstructionModel, SLOT(refresh()));

    connect(this, SIGNAL(changed()), this, SIGNAL(cameraCalibrationModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(rigCalibrationModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(recordingModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(reconstructionModelChanged()));
}

bool Project::open(const QString& path)
{
    QString db_path;
    bool ok = true;

    if(ok)
    {
        ok = mDir.cd(path);
    }

    if(ok)
    {
        db_path = mDir.absoluteFilePath("db.sqlite");
        ok = QFileInfo::exists(db_path);
    }

    if(ok)
    {
        mDB = QSqlDatabase::database();
        ok = mDB.isValid();
    }

    if(ok)
    {
        mDB.setDatabaseName(db_path);
        ok = mDB.open();
    }

    if(ok)
    {
        QStringList tables = mDB.tables();
        ok = tables.contains("reconstructions"); // we check only one table. we could check more.
    }

    if(ok == false)
    {
        close();
    }

    changed();

    return ok;
}

void Project::close()
{
    mDir = QDir();
    mDB.close();

    changed();
}

void Project::beginTransaction()
{
    mDB.transaction();
}

void Project::endTransaction()
{
    mDB.commit();
}

void Project::abortTransaction()
{
    mDB.rollback();
}

CameraCalibrationModel* Project::cameraCalibrationModel()
{
    return mCameraCalibrationModel;
}

RigCalibrationModel* Project::rigCalibrationModel()
{
    return mRigCalibrationModel;
}

RecordingModel* Project::recordingModel()
{
    return mRecordingModel;
}

ReconstructionModel* Project::reconstructionModel()
{
    return mReconstructionModel;
}

bool Project::saveCamera(CameraCalibrationDataPtr camera, int& id)
{
    bool ok = mDB.isOpen();;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO 'camera_parameters' ('name','date','fx','fy','cx','cy','distortion_model') VALUES (?, DATETIME('NOW'), ?, ?, ?, ?, 0)");
        q.addBindValue(camera->name.c_str());
        q.addBindValue(camera->calibration_matrix.at<double>(0,0));
        q.addBindValue(camera->calibration_matrix.at<double>(1,1));
        q.addBindValue(camera->calibration_matrix.at<double>(0,2));
        q.addBindValue(camera->calibration_matrix.at<double>(1,2));

        const bool ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        const cv::Mat dist = camera->distortion_coefficients;

        for(int j=0; ok && j<dist.cols; j++)
        {
            QSqlQuery q(mDB);
            q.prepare("INSERT INTO 'distortion_coefficients' ('camera_id', 'rank', 'value') VALUES (?,?,?)");
            q.addBindValue(id);
            q.addBindValue(j);
            q.addBindValue(dist.at<double>(0, j));

            ok = q.exec();
        }
    }

    return ok;
}

bool Project::loadCamera(int id, CameraCalibrationDataPtr& camera)
{
    bool ok = mDB.isOpen();;

    camera.reset(new CameraCalibrationData());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT 'name', 'date', 'fx', 'fy', 'cx', 'cy', 'distortion_model' FROM 'camera_parameters' WHERE id=?");
        q.addBindValue(id);

        bool ok = q.exec() && q.next();

        if(ok)
        {
            camera->name = q.value(0).toString().toStdString();

            camera->calibration_matrix = cv::Mat(3, 3, CV_64F);
            camera->calibration_matrix.at<double>(0,0) = q.value(2).toDouble();
            camera->calibration_matrix.at<double>(1,1) = q.value(3).toDouble();
            camera->calibration_matrix.at<double>(0,2) = q.value(4).toDouble();
            camera->calibration_matrix.at<double>(1,2) = q.value(5).toDouble();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT 'rank', 'value' FROM 'distortion_coefficients' WHERE 'camera_id'=? ORDER BY 'rank' ASC");
        q.addBindValue(id);

        std::vector<double> values;

        ok = q.exec();

        if(ok)
        {
            while(ok && q.next())
            {
                const int rank = q.value(0).toInt();
                ok = (values.size() == rank);
                values.push_back( q.value(1).toDouble() );
            }
        }

        if(ok)
        {
            camera->distortion_coefficients = cv::Mat(1, values.size(), CV_64F);
            for(size_t j=0; j<values.size(); j++)
            {
                camera->distortion_coefficients.at<double>(j, 0) = values[j];
            }
        }
    }

    if(ok == false)
    {
        camera.reset();
    }

    return ok;
}

bool Project::savePose(const Sophus::SE3d& pose, int& id)
{
    bool ok = mDB.isOpen();;

    if(ok)
    {
        Eigen::Quaterniond r = pose.unit_quaternion();
        Eigen::Vector3d t = pose.translation();

        QSqlQuery q(mDB);
        q.prepare("INSERT INTO poses(qx, qy, qz, qw, x, y, z) VALUES(?,?,?,?,?,?,?)");
        q.addBindValue(r.x());
        q.addBindValue(r.y());
        q.addBindValue(r.z());
        q.addBindValue(r.w());
        q.addBindValue(t.x());
        q.addBindValue(t.y());
        q.addBindValue(t.z());

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    return ok;
}

bool Project::loadPose(int id, Sophus::SE3d& pose)
{
    bool ok = mDB.isOpen();;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT qx, qy, qz, qw, x, y, z FROM poses WHERE id=?");
        q.addBindValue(id);

        ok = ( q.exec() && q.next() );

        if(ok)
        {
            Eigen::Quaterniond r;
            Eigen::Vector3d t;

            r.x() = q.value(0).toFloat();
            r.y() = q.value(1).toFloat();
            r.z() = q.value(2).toFloat();
            r.w() = q.value(3).toFloat();
            t.x() = q.value(4).toFloat();
            t.y() = q.value(5).toFloat();
            t.z() = q.value(6).toFloat();

            pose.setQuaternion(r);
            pose.translation() = t;
        }
    }

    if(ok == false)
    {
        pose = Sophus::SE3d();
    }

    return ok;
}

bool Project::saveRig(StereoRigCalibrationDataPtr rig, int& id)
{
    return false; // TODO
}

bool Project::loadRig(int id, StereoRigCalibrationDataPtr& rig)
{
    return false; // TODO
}

bool Project::listCameras(CameraCalibrationList& list)
{
    bool ok = mDB.isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = q.exec("SELECT id, name, date FROM camera_parameters");

        if(ok)
        {
            while(q.next())
            {
                CameraCalibrationListItem item;
                item.id = q.value(0).toInt();
                item.name = q.value(1).toString();
                item.date = q.value(2).toString();
                list.push_back(item);
            }
        }
    }

    if(ok == false)
    {
        list.clear();
    }

    return ok;
}

bool Project::listRigs(RigCalibrationList& list)
{
    bool ok = mDB.isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = q.exec("SELECT id, name, date FROM rig_parameters");

        if(ok)
        {
            while(q.next())
            {
                RigCalibrationListItem item;
                item.id = q.value(0).toInt();
                item.name = q.value(1).toString();
                item.date = q.value(2).toString();
                list.push_back(item);
            }
        }
    }

    if(ok == false)
    {
        list.clear();
    }

    return ok;
}

bool Project::listRecordings(RecordingList& list)
{
    bool ok = mDB.isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = q.exec("SELECT id, name, date FROM recordings");

        if(ok)
        {
            while(q.next())
            {
                RecordingListItem item;
                item.id = q.value(0).toInt();
                item.name = q.value(1).toString();
                item.date = q.value(2).toString();
                list.push_back(item);
            }
        }
    }

    if(ok == false)
    {
        list.clear();
    }

    return ok;
}

bool Project::listReconstructions(ReconstructionList& list)
{
    bool ok = mDB.isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = q.exec("SELECT id, name, date FROM reconstructions");

        if(ok)
        {
            while(q.next())
            {
                ReconstructionListItem item;
                item.id = q.value(0).toInt();
                item.name = q.value(1).toString();
                item.date = q.value(2).toString();
                list.push_back(item);
            }
        }
    }

    if(ok == false)
    {
        list.clear();
    }

    return ok;
}

void Project::clear()
{
    // TODO
}

