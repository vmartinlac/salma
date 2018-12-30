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

bool Project::create(const QString& path)
{
    close();

    // TODO
    std::cout << "Not implemented yet!" << std::endl;

    return false;
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

bool Project::clear()
{
    mDB.exec("DELETE FROM `poses`");
    mDB.exec("DELETE FROM `camera_parameters`");
    mDB.exec("DELETE FROM `distortion_coefficients`");
    mDB.exec("DELETE FROM `rig_parameters`");
    mDB.exec("DELETE FROM `rig_cameras`");
    mDB.exec("DELETE FROM `recordings`");
    mDB.exec("DELETE FROM `recording_frames`");
    mDB.exec("DELETE FROM `recording_views`");
    mDB.exec("DELETE FROM `reconstructions`");
    mDB.exec("DELETE FROM `settings`");
    mDB.exec("DELETE FROM `frames`");
    mDB.exec("DELETE FROM `views`");
    mDB.exec("DELETE FROM `mappoints`");
    mDB.exec("DELETE FROM `keypoints`");
    mDB.exec("DELETE FROM `descriptors`");
    mDB.exec("DELETE FROM `projections`");
    mDB.exec("DELETE FROM `densepoints`");

    changed();

    return true;
}

bool Project::isOpen()
{
    return mDB.isOpen();
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

// CAMERA CALIBRATION

bool Project::saveCamera(CameraCalibrationDataPtr camera, int& id)
{
    bool ok = isOpen();

    if(ok)
    {
        ok = (camera->id < 0);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO `camera_parameters` (`name`,`date`,`fx`,`fy`,`cx`,`cy`,`distortion_model`, `image_width`, `image_height`) VALUES (?, DATETIME('NOW'), ?, ?, ?, ?, 0, ?, ?)");
        q.addBindValue(camera->name.c_str());
        q.addBindValue(camera->calibration_matrix.at<double>(0,0));
        q.addBindValue(camera->calibration_matrix.at<double>(1,1));
        q.addBindValue(camera->calibration_matrix.at<double>(0,2));
        q.addBindValue(camera->calibration_matrix.at<double>(1,2));
        q.addBindValue(camera->image_size.width);
        q.addBindValue(camera->image_size.height);

        const bool ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
            camera->id = id;
        }
    }

    if(ok)
    {
        const cv::Mat dist = camera->distortion_coefficients;

        for(int j=0; ok && j<dist.cols; j++)
        {
            QSqlQuery q(mDB);
            q.prepare("INSERT INTO `distortion_coefficients` (`camera_id`, `rank`, `value`) VALUES (?,?,?)");
            q.addBindValue(id);
            q.addBindValue(j);
            q.addBindValue(dist.at<double>(0, j));

            ok = q.exec();
        }
    }

    if(ok == false)
    {
        camera->id = -1;
    }

    cameraCalibrationModelChanged();

    return ok;
}

bool Project::describeCamera(int id, QString& descr)
{
    bool ok = isOpen();

    CameraCalibrationDataPtr camera;

    descr.clear();

    if(ok)
    {
        ok = loadCamera(id, camera) && bool(camera);
    }

    if(ok)
    {
        std::stringstream s;
        s << "<html><head></head><body>" << std::endl;

        s << "<h3>Metadata</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
        s << "<tr><th>name</th><td>" << camera->name << "</td></tr>" << std::endl;
        s << "<tr><th>date</th><td>" << camera->date << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "<h3>Image resolution</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>width</th><td>" << camera->image_size.width << "</td></tr>" << std::endl;
        s << "<tr><th>height</th><td>" << camera->image_size.height << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "<h3>Pinhole model</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>fx</th><td>" << camera->calibration_matrix.at<double>(0,0) << "</td></tr>" << std::endl;
        s << "<tr><th>fy</th><td>" << camera->calibration_matrix.at<double>(1,1) << "</td></tr>" << std::endl;
        s << "<tr><th>cx</th><td>" << camera->calibration_matrix.at<double>(0,2) << "</td></tr>" << std::endl;
        s << "<tr><th>cy</th><td>" << camera->calibration_matrix.at<double>(1,2) << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "<h3>Lens distortion model</h3>" << std::endl;
        s << "<table>" << std::endl;
        for(int i=0; i<camera->distortion_coefficients.cols; i++)
        {
            s << "<tr><th>" << i << "</th><td>" << camera->distortion_coefficients.at<double>(0,i) << "</td></tr>" << std::endl;
        }
        s << "</table>" << std::endl;

        s << "</body></html>" << std::endl;

        descr = s.str().c_str();
    }

    return ok;
}

bool Project::renameCamera(int id, const QString& new_name)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("UPDATE camera_parameters SET name=? WHERE id=?");
        q.addBindValue(new_name);
        q.addBindValue(id);

        ok = q.exec() && (q.numRowsAffected() >= 1);
    }

    cameraCalibrationModelChanged();

    return ok;
}

bool Project::listCameras(CameraCalibrationList& list)
{
    bool ok = isOpen();

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

bool Project::loadCamera(int id, CameraCalibrationDataPtr& camera)
{
    bool ok = isOpen();

    camera.reset(new CameraCalibrationData());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `fx`, `fy`, `cx`, `cy`, `distortion_model`, `image_width`, `image_height` FROM `camera_parameters` WHERE id=?");
        q.addBindValue(id);

        bool ok = q.exec() && q.next();

        if(ok)
        {
            camera->id = id;

            camera->name = q.value(0).toString().toStdString();
            camera->date = q.value(1).toString().toStdString();

            camera->image_size.width = q.value(7).toInt();
            camera->image_size.height = q.value(8).toInt();

            camera->calibration_matrix.create(3, 3, CV_64F);
            camera->calibration_matrix.at<double>(0,0) = q.value(2).toDouble();
            camera->calibration_matrix.at<double>(0,1) = 0.0;
            camera->calibration_matrix.at<double>(0,2) = q.value(4).toDouble();
            camera->calibration_matrix.at<double>(1,0) = 0.0;
            camera->calibration_matrix.at<double>(1,1) = q.value(3).toDouble();
            camera->calibration_matrix.at<double>(1,2) = q.value(5).toDouble();
            camera->calibration_matrix.at<double>(2,0) = 0.0;
            camera->calibration_matrix.at<double>(2,1) = 0.0;
            camera->calibration_matrix.at<double>(2,2) = 1.0;
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `rank`, `value` FROM `distortion_coefficients` WHERE `camera_id`=? ORDER BY `rank` ASC");
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
            camera->distortion_coefficients.create(1, values.size(), CV_64F);

            for(size_t j=0; j<values.size(); j++)
            {
                camera->distortion_coefficients.at<double>(0, j) = values[j];
            }
        }
    }

    if(ok == false)
    {
        camera.reset();
    }

    return ok;
}

// POSE

bool Project::savePose(const Sophus::SE3d& pose, int& id)
{
    bool ok = isOpen();

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
    bool ok = isOpen();

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

// RIG CALIBRATION

bool Project::saveRig(StereoRigCalibrationDataPtr rig, int& id)
{
    bool ok = true;

    if(ok)
    {
        ok = (rig->id < 0);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO `rig_parameters` (`name`, `date`, `number_of_cameras`) VALUES (?, DATETIME('NOW'), 2)");
        q.addBindValue(rig->name.c_str());

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
            rig->id = id;
        }
    }

    for(int i=0; ok && i<2; i++)
    {
        int pose_id = -1;

        if(ok)
        {
            ok = savePose(rig->cameras[i].camera_to_rig, pose_id);
        }

        if(ok)
        {
            QSqlQuery q(mDB);
            q.prepare("INSERT INTO `rig_cameras` (`rig_id`, `rank`, `camera_to_rig`, `camera_id`) VALUES(?,?,?,?)");
            q.addBindValue(id);
            q.addBindValue(i);
            q.addBindValue(pose_id);
            q.addBindValue(rig->cameras[i].calibration->id);

            ok = q.exec();
        }
    }

    rigCalibrationModelChanged();

    return ok;
}

bool Project::describeRig(int id, QString& descr)
{
    bool ok = isOpen();

    StereoRigCalibrationDataPtr rig;

    descr.clear();

    if(ok)
    {
        ok = loadRig(id, rig) && bool(rig);
    }

    if(ok)
    {
        std::stringstream s;
        s << "<html><head></head><body>" << std::endl;

        s << "<h3>Metadata</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
        s << "<tr><th>name</th><td>" << rig->name << "</td></tr>" << std::endl;
        s << "<tr><th>date</th><td>" << rig->date << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        for(int i=0; i<2; i++)
        {
            s << "<h3>Camera #" << i << "</h3>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>Camera calibration</th><td><em>" << QString(rig->cameras[i].calibration->name.c_str()).toHtmlEscaped().toStdString() << "</em> (" << rig->cameras[i].calibration->id << ").</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_tx</th><td>" << rig->cameras[i].camera_to_rig.translation().x() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_ty</th><td>" << rig->cameras[i].camera_to_rig.translation().y() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_tz</th><td>" << rig->cameras[i].camera_to_rig.translation().z() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qx</th><td>" << rig->cameras[i].camera_to_rig.unit_quaternion().x() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qy</th><td>" << rig->cameras[i].camera_to_rig.unit_quaternion().y() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qz</th><td>" << rig->cameras[i].camera_to_rig.unit_quaternion().z() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qw</th><td>" << rig->cameras[i].camera_to_rig.unit_quaternion().w() << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;
        }

        s << "</body></html>" << std::endl;

        descr = s.str().c_str();
    }

    return ok;
}

bool Project::loadRig(int id, StereoRigCalibrationDataPtr& rig)
{
    bool ok = isOpen();

    rig.reset(new StereoRigCalibrationData());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `number_of_cameras` FROM `rig_parameters` WHERE `id`=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            ok = (q.value(2).toInt() == 2);
        }

        if(ok)
        {
            rig->id = id;
            rig->name = q.value(0).toString().toStdString();
            rig->date = q.value(1).toString().toStdString();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `rank`, `camera_to_rig`, `camera_id` FROM `rig_cameras` WHERE `rig_id`=? ORDER BY `rank` ASC");
        q.addBindValue(id);
        ok = q.exec();

        std::vector<StereoRigCalibrationDataCamera> cameras;

        if(ok)
        {
            while(ok && q.next())
            {
                StereoRigCalibrationDataCamera item;

                if(ok)
                {
                    ok = ( q.value(0).toInt() == cameras.size() );
                }

                if(ok)
                {
                    ok = loadPose( q.value(1).toInt(), item.camera_to_rig);
                }

                if(ok)
                {
                    ok = loadCamera( q.value(2).toInt(), item.calibration );
                }

                if(ok)
                {
                    cameras.push_back(std::move(item));
                }
            }
        }

        if(ok)
        {
            ok = ( cameras.size() == 2 );
        }

        if(ok)
        {
            rig->cameras[0] = std::move(cameras[0]);
            rig->cameras[1] = std::move(cameras[1]);
        }
    }

    if(ok == false)
    {
        rig.reset();
    }

    return ok;
}

bool Project::renameRig(int id, const QString& new_name)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("UPDATE rig_parameters SET name=? WHERE id=?");
        q.addBindValue(new_name);
        q.addBindValue(id);

        ok = q.exec() && (q.numRowsAffected() >= 1);
    }

    rigCalibrationModelChanged();

    return ok;
}

bool Project::listRigs(RigCalibrationList& list)
{
    bool ok = isOpen();

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

// RECORDING

bool Project::saveRecording(RecordingHeaderPtr rec, int& id)
{
    return false; // TODO
}

bool Project::loadRecording(int id, RecordingHeaderPtr& rec)
{
    bool ok = isOpen();

    rec.reset(new RecordingHeader());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtome'), `directory`, `number_of_views` FROM `recordings` WHERE `id`=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            rec->id = id;
            rec->name = q.value(0).toString().toStdString();
            rec->date = q.value(1).toString().toStdString();

            rec->directory = mDir;
            ok = rec->directory.cd(q.value(2).toString());

            rec->num_views = q.value(3).toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT recording_frames.time, recording_views.filename, recording_frames.rank, recording_views.view FROM recording_views, recording_frames WHERE recording_views.frame_id=recording_frames.id AND recording_frames.recording_id=? ORDER BY recording_frames.rank ASC, recording_views.view ASC");
        q.addBindValue(id);
        ok = q.exec();

        while( ok && q.next() )
        {
            ; // TODO
        }
    }

    if(ok == false)
    {
        rec.reset();
    }

    return ok;
}

bool Project::isRecordingMutable(int id, bool& mut)
{
    return false; // TODO
}

bool Project::describeRecording(int id, QString& descr)
{
    bool ok = true;

    double duration = 0.0;
    int num_frames = 0;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT MAX(`rank`)+1 AS `number_of_frames`, MAX(`time`) AS `duration` FROM recording_frames WHERE recording_id=?");
        q.addBindValue(id);
        ok = q.exec();
        
        if(ok)
        {
            if(q.next())
            {
                num_frames = q.value(0).toInt();
                duration = q.value(1).toDouble();
            }
            else
            {
                num_frames = 0;
                duration = 0.0;
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `directory`, `number_of_views` FROM `recordings` WHERE `id`=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            std::stringstream s;

            s << "<html><head></head><body>" << std::endl;

            s << "<h3>Metadata</h3>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
            s << "<tr><th>name</th><td>" << q.value(0).toString().toStdString() << "</td></tr>" << std::endl;
            s << "<tr><th>date</th><td>" << q.value(1).toString().toStdString() << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            s << "<h3>Content</h3>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>Number of cameras:</th><td>" << q.value(3).toInt() << "</td></tr>" << std::endl;
            s << "<tr><th>Directory:</th><td>" << q.value(2).toString().toStdString() << "</td></tr>" << std::endl;
            s << "<tr><th>Number of frames:</th><td>" << num_frames << "</td></tr>" << std::endl;
            s << "<tr><th>Duration:</th><td>" << duration << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            s << "</body></html>" << std::endl;

            descr = s.str().c_str();
        }
    }

    return ok;
}

bool Project::renameRecording(int id, const QString& new_name)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("UPDATE recordings SET name=? WHERE id=?");
        q.addBindValue(new_name);
        q.addBindValue(id);

        ok = q.exec() && (q.numRowsAffected() >= 1);
    }

    recordingModelChanged();

    return ok;
}

bool Project::listRecordings(RecordingList& list)
{
    bool ok = isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = q.exec("SELECT `id`, `name`, `date` FROM `recordings`");

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

// RECONSTRUCTION

bool Project::listReconstructions(ReconstructionList& list)
{
    bool ok = isOpen();

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

