#include <QSqlQuery>
#include <QTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QFileInfo>
#include "Project.h"

#define PROJECT_DATABASE_NAME "db.sqlite"

Project::Project(QObject* parent) : QObject(parent)
{
    mCalibrationModel = new CalibrationModel(this);
    mRecordingModel = new RecordingModel(this);
    mReconstructionModel = new ReconstructionModel(this);

    connect(this, SIGNAL(recordingModelChanged()), mRecordingModel, SLOT(refresh()));
    connect(this, SIGNAL(calibrationModelChanged()), mCalibrationModel, SLOT(refresh()));
    connect(this, SIGNAL(reconstructionModelChanged()), mReconstructionModel, SLOT(refresh()));

    connect(this, SIGNAL(changed()), this, SIGNAL(recordingModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(calibrationModelChanged()));
    connect(this, SIGNAL(changed()), this, SIGNAL(reconstructionModelChanged()));
}

bool Project::create(const QString& path)
{
    QString db_path;
    QFile file;
    QStringList statements;
    bool ok = true;

    close();

    // open project directory.

    if(ok)
    {
        mDir = QDir();
        ok = mDir.cd(path);
    }

    // check if there is any existing db.

    if(ok)
    {
        db_path = mDir.absoluteFilePath(PROJECT_DATABASE_NAME);
        ok = ( QFileInfo::exists(db_path) == false );
    }

    // open sqlite database.

    if(ok)
    {
        mDB = QSqlDatabase::database();
        mDB.setDatabaseName(db_path);
        ok = mDB.open();
    }

    // create the tables.

    if(ok)
    {
        file.setFileName(":/db.sql");
        ok = file.open(QIODevice::ReadOnly);
    }

    if(ok)
    {
        QString content = QString::fromUtf8(file.readAll());
        statements = content.split(";", QString::SkipEmptyParts);

        ok = mDB.transaction();

        for(QStringList::iterator it=statements.begin(); ok && it!=statements.end(); it++)
        {
            if( it->contains("CREATE", Qt::CaseInsensitive) )
            {
                QSqlQuery q(mDB);
                ok = q.exec(*it);
            }
        }

        if(ok)
        {
            ok = mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    if( file.isOpen() )
    {
        file.close();
    }

    if(ok == false)
    {
        close();
    }

    return ok;
}

bool Project::open(const QString& path)
{
    QString db_path;
    bool ok = true;

    if(ok)
    {
        mDir = QDir();
        ok = mDir.cd(path);
    }

    if(ok)
    {
        db_path = mDir.absoluteFilePath(PROJECT_DATABASE_NAME);
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
    bool has_transaction = false;
    bool ok = true;

    if(ok)
    {
        ok = mDB.transaction();
        if(ok)
        {
            has_transaction = true;
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        ok = ok && q.exec("DELETE FROM `poses`");
        ok = ok && q.exec("DELETE FROM `cameras`");
        ok = ok && q.exec("DELETE FROM `distortion_coefficients`");
        ok = ok && q.exec("DELETE FROM `rigs`");
        ok = ok && q.exec("DELETE FROM `recordings`");
        ok = ok && q.exec("DELETE FROM `recording_frames`");
        ok = ok && q.exec("DELETE FROM `recording_views`");
        ok = ok && q.exec("DELETE FROM `reconstructions`");
        ok = ok && q.exec("DELETE FROM `settings`");
        ok = ok && q.exec("DELETE FROM `frames`");
        ok = ok && q.exec("DELETE FROM `views`");
        ok = ok && q.exec("DELETE FROM `mappoints`");
        ok = ok && q.exec("DELETE FROM `keypoints`");
        ok = ok && q.exec("DELETE FROM `descriptors`");
        ok = ok && q.exec("DELETE FROM `projections`");
        ok = ok && q.exec("DELETE FROM `densepoints`");
    }

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    changed();

    return ok;
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

CalibrationModel* Project::calibrationModel()
{
    return mCalibrationModel;
}

RecordingModel* Project::recordingModel()
{
    return mRecordingModel;
}

ReconstructionModel* Project::reconstructionModel()
{
    return mReconstructionModel;
}

// CALIBRATION

bool Project::saveCalibration(StereoRigCalibrationPtr rig, int& id)
{
    bool has_transaction = false;
    bool ok = true;

    std::vector<int> pose_ids;
    std::vector<int> camera_ids;

    if(ok)
    {
        ok = (rig->id < 0);
    }

    if(ok)
    {
        ok = mDB.transaction();
        if(ok)
        {
            has_transaction = true;
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO `rigs` (`name`, `date`) VALUES (?, DATETIME('NOW'))");
        q.bindValue(0, rig->name.c_str());

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        pose_ids.resize(rig->cameras.size(), -1);

        QSqlQuery q(mDB);
        q.prepare("INSERT INTO poses(qx, qy, qz, qw, x, y, z) VALUES(?,?,?,?,?,?,?)");

        for(int i=0; ok && i<rig->cameras.size(); i++)
        {
            const Sophus::SE3d& pose = rig->cameras[i].camera_to_rig;
            const Eigen::Quaterniond r = pose.unit_quaternion();
            const Eigen::Vector3d t = pose.translation();

            q.bindValue(0, r.x());
            q.bindValue(1, r.y());
            q.bindValue(2, r.z());
            q.bindValue(3, r.w());
            q.bindValue(4, t.x());
            q.bindValue(5, t.y());
            q.bindValue(6, t.z());

            ok = q.exec();

            if(ok)
            {
                pose_ids[i] = q.lastInsertId().toInt();
            }
        }
    }

    if(ok)
    {
        camera_ids.resize(rig->cameras.size(), -1);

        QSqlQuery q(mDB);

        q.prepare(
            "INSERT INTO cameras (rig_id, rank_in_rig, image_width, image_height, fx, fy, cx, cy, distortion_model, camera_to_rig) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0, ?)"
        );

        for(int i=0; ok && i<rig->cameras.size(); i++)
        {
            const CameraCalibration& camera = rig->cameras[i];

            q.bindValue(0, id);
            q.bindValue(1, i);
            q.bindValue(2, camera.image_size.width);
            q.bindValue(3, camera.image_size.height);
            q.bindValue(4, camera.calibration_matrix.at<double>(0,0));
            q.bindValue(5, camera.calibration_matrix.at<double>(1,1));
            q.bindValue(6, camera.calibration_matrix.at<double>(0,2));
            q.bindValue(7, camera.calibration_matrix.at<double>(1,2));
            q.bindValue(8, pose_ids[i]);

            const bool ok = q.exec();

            if(ok)
            {
                camera_ids[i] = q.lastInsertId().toInt();
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO `distortion_coefficients` (`camera_id`, `rank`, `value`) VALUES (?,?,?)");

        for(int i=0; ok && i<rig->cameras.size(); i++)
        {
            const cv::Mat dist = rig->cameras[i].distortion_coefficients;

            for(int j=0; ok && j<dist.cols; j++)
            {
                q.bindValue(0, camera_ids[i]);
                q.bindValue(1, j);
                q.bindValue(2, dist.at<double>(0, j));

                ok = q.exec();
            }
        }
    }

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    if(ok)
    {
        rig->id = id;
    }

    calibrationModelChanged();

    return ok;
}

bool Project::describeCalibration(int id, QString& descr)
{
    bool ok = isOpen();

    StereoRigCalibrationPtr rig;

    descr.clear();

    if(ok)
    {
        ok = loadCalibration(id, rig) && bool(rig);
    }

    if(ok)
    {
        std::stringstream s;
        s << "<html><head></head><body>" << std::endl;

        s << "<h3>Metadata</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
        s << "<tr><th>name</th><td>" << htmlEscape(rig->name) << "</td></tr>" << std::endl;
        s << "<tr><th>date</th><td>" << htmlEscape(rig->date) << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        int rank = 0;
        for(CameraCalibration& cam : rig->cameras)
        {
            s << "<h3>Camera #" << rank << "</h3>" << std::endl;

            s << "<h4>Image resolution</h4>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>width</th><td>" << cam.image_size.width << "</td></tr>" << std::endl;
            s << "<tr><th>height</th><td>" << cam.image_size.height << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            s << "<h4>Pinhole model</h4>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>fx</th><td>" << cam.calibration_matrix.at<double>(0,0) << "</td></tr>" << std::endl;
            s << "<tr><th>fy</th><td>" << cam.calibration_matrix.at<double>(1,1) << "</td></tr>" << std::endl;
            s << "<tr><th>cx</th><td>" << cam.calibration_matrix.at<double>(0,2) << "</td></tr>" << std::endl;
            s << "<tr><th>cy</th><td>" << cam.calibration_matrix.at<double>(1,2) << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            s << "<h4>Lens distortion model</h4>" << std::endl;
            if( cam.distortion_coefficients.cols == 0 )
            {
                s << "<p>No distortion.</p>" << std::endl;
            }
            else
            {
                s << "<table>" << std::endl;
                for(int i=0; i<cam.distortion_coefficients.cols; i++)
                {
                    s << "<tr><th>" << i << "</th><td>" << cam.distortion_coefficients.at<double>(0,i) << "</td></tr>" << std::endl;
                }
                s << "</table>" << std::endl;
            }

            s << "<h4>Pose</h4>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>camera_to_rig_tx</th><td>" << cam.camera_to_rig.translation().x() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_ty</th><td>" << cam.camera_to_rig.translation().y() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_tz</th><td>" << cam.camera_to_rig.translation().z() << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>camera_to_rig_qx</th><td>" << cam.camera_to_rig.unit_quaternion().x() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qy</th><td>" << cam.camera_to_rig.unit_quaternion().y() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qz</th><td>" << cam.camera_to_rig.unit_quaternion().z() << "</td></tr>" << std::endl;
            s << "<tr><th>camera_to_rig_qw</th><td>" << cam.camera_to_rig.unit_quaternion().w() << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            rank++;
        }

        s << "</body></html>" << std::endl;

        descr = s.str().c_str();
    }

    return ok;
}

bool Project::loadCalibration(int id, StereoRigCalibrationPtr& rig)
{
    QTime time;
    time.start();

    bool ok = isOpen();

    rig.reset(new StereoRigCalibration());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime') FROM `rigs` WHERE `id`=?");

        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

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
        q.prepare(
            "SELECT c.id, c.rank_in_rig, c.image_width, c.image_height, c.fx, c.fy, c.cx, c.cy, c.distortion_model, p.qx, p.qy, p.qz, p.qw, p.x, p.y, p.z "
            "FROM cameras c, poses p "
            "WHERE p.id=c.camera_to_rig AND c.rig_id=?"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            const int rank = q.value(1).toInt();

            if(ok)
            {
                ok = (rank == 0 || rank == 1) && (q.value(8).toInt() == 0);
            }

            if(ok)
            {
                CameraCalibration& camera = rig->cameras[rank];

                Eigen::Quaterniond r;
                Eigen::Vector3d t;

                r.x() = q.value(9).toFloat();
                r.y() = q.value(10).toFloat();
                r.z() = q.value(11).toFloat();
                r.w() = q.value(12).toFloat();
                t.x() = q.value(13).toFloat();
                t.y() = q.value(14).toFloat();
                t.z() = q.value(15).toFloat();

                camera.camera_to_rig.setQuaternion(r);
                camera.camera_to_rig.translation() = t;

                camera.image_size.width = q.value(2).toInt();
                camera.image_size.height = q.value(3).toInt();

                camera.calibration_matrix.create(3, 3, CV_64F);
                camera.calibration_matrix.at<double>(0,0) = q.value(4).toDouble();
                camera.calibration_matrix.at<double>(0,1) = 0.0;
                camera.calibration_matrix.at<double>(0,2) = q.value(6).toDouble();
                camera.calibration_matrix.at<double>(1,0) = 0.0;
                camera.calibration_matrix.at<double>(1,1) = q.value(5).toDouble();
                camera.calibration_matrix.at<double>(1,2) = q.value(7).toDouble();
                camera.calibration_matrix.at<double>(2,0) = 0.0;
                camera.calibration_matrix.at<double>(2,1) = 0.0;
                camera.calibration_matrix.at<double>(2,2) = 1.0;
            }
        }
    }

    if(ok)
    {
        std::array< std::vector<double>, 2> coeffs;

        QSqlQuery q(mDB);
        q.prepare(
            "SELECT ca.rank_in_rig, co.rank, co.value FROM distortion_coefficients co, cameras ca "
            "WHERE ca.rig_id=? AND co.camera_id=ca.id "
            "ORDER BY ca.rank_in_rig ASC, co.rank ASC"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            const int camera_rank = q.value(0).toInt();
            const int coefficient_rank = q.value(1).toInt();

            if(ok)
            {
                ok = (camera_rank == 0 || camera_rank == 1);
            }

            if(ok)
            {
                ok = (coeffs[camera_rank].size() == coefficient_rank);
            }

            if(ok)
            {
                coeffs[camera_rank].push_back( q.value(2).toDouble() );
            }
        }

        for(int i=0; i<2; i++)
        {
            rig->cameras[i].distortion_coefficients.create(1, coeffs[i].size(), CV_64F);

            for(size_t j=0; j<coeffs[i].size(); j++)
            {
                rig->cameras[i].distortion_coefficients.at<double>(0, j) = coeffs[i][j];
            }
        }
    }

    if(ok)
    {
        const int msec = time.elapsed();
        std::cout << "Calibration loaded in " << msec << " milliseconds." << std::endl;
    }
    else
    {
        rig.reset();
    }

    /*
    if(ok)
    {
        QJsonDocument doc( rig->toJson().toObject() );
        std::cout << doc.toJson().toStdString() << std::endl;
    }
    */

    return ok;
}

bool Project::renameCalibration(int id, const QString& new_name)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("UPDATE rigs SET name=? WHERE id=?");
        q.bindValue(0, new_name);
        q.bindValue(1, id);

        ok = q.exec() && (q.numRowsAffected() >= 1);
    }

    calibrationModelChanged();

    return ok;
}

bool Project::removeCalibration(int id)
{
    bool ok = true;
    bool has_transaction = false;

    // check whether the rig can be removed or not.

    if(ok)
    {
        bool ismutable;
        ok = ( isCalibrationMutable(id, ismutable) && ismutable );
    }

    // create transaction.

    if(ok)
    {
        ok = mDB.transaction();

        if(ok)
        {
            has_transaction = true;
        }
    }

    // delete rig camera poses.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM poses WHERE id IN (SELECT camera_to_rig FROM cameras WHERE rig_id=?)");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete distortion coefficients.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM distortion_coefficients WHERE camera_id IN (SELECT id FROM cameras WHERE rig_id=?)");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete cameras.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM cameras WHERE rig_id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete rig.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM rigs WHERE id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // commit or rollback transaction.

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    calibrationModelChanged();

    return ok;
}

bool Project::isCalibrationMutable(int id, bool& ismutable)
{
    bool ok = true;

    ismutable = true;

    // check that there is such rig in the databse.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM rigs WHERE id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();
    }

    // check whether some reconstruction is referencing the rig.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(DISTINCT id) FROM reconstructions WHERE rig_id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);

        ok = q.exec() && q.next();

        if(ok)
        {
            ismutable = (q.value(0).toInt() == 0);
        }
    }

    if(ok == false)
    {
        ismutable = false;
    }

    return ok;
}

bool Project::listCalibrations(CalibrationList& list)
{
    bool ok = isOpen();

    list.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.setForwardOnly(true);
        ok = q.exec("SELECT id, name, date FROM rigs");

        if(ok)
        {
            while(q.next())
            {
                CalibrationListItem item;
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
    bool has_transaction = false;
    bool ok = true;

    if(ok)
    {
        ok = ( rec->id < 0 );
    }

    if(ok)
    {
        ok = mDB.transaction();

        if(ok)
        {
            has_transaction = true;
        }
    }

    if(ok)
    {
        const QString dir_name = rec->directory.dirName();

        QSqlQuery q(mDB);
        q.prepare("INSERT INTO recordings(name, date, directory) VALUES(?,DATETIME('now'),?)");
        q.bindValue(0, rec->name.c_str());
        q.bindValue(1, dir_name);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);

        q.prepare("INSERT INTO recording_views(recording_id, rank, width, height) VALUES(?,?,?,?)");

        for(int i=0; ok && i<rec->num_views(); i++)
        {
            q.bindValue(0, id);
            q.bindValue(1, i);
            q.bindValue(2, rec->views[i].width);
            q.bindValue(3, rec->views[i].height);

            ok = q.exec();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);

        q.prepare("INSERT INTO recording_frames(recording_id, rank, timestamp) VALUES(?,?,?)");

        for(int i=0; ok && i<rec->num_frames(); i++)
        {
            q.bindValue(0, id);
            q.bindValue(1, i);
            q.bindValue(2, rec->timestamps[i]);

            ok = q.exec();
        }
    }

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    if(ok)
    {
        rec->id = id;
    }

    recordingModelChanged();

    return ok;
}

bool Project::loadRecording(int id, RecordingHeaderPtr& rec)
{
    QTime time;
    time.start();

    bool ok = isOpen();

    rec.reset(new RecordingHeader());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `directory` FROM `recordings` WHERE `id`=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            rec->id = id;
            rec->name = q.value(0).toString().toStdString();
            rec->date = q.value(1).toString().toStdString();
            
            rec->directory = mDir;
            ok = rec->directory.cd( q.value(2).toString() );
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT rank, width, height FROM recording_views WHERE recording_id=? ORDER BY rank ASC");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while( ok && q.next() )
        {
            ok = ( q.value(0).toInt() == rec->views.size() );

            if(ok)
            {
                rec->views.emplace_back(
                    q.value(1).toDouble(),
                    q.value(2).toDouble());
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT rank, timestamp FROM recording_frames WHERE recording_id=? ORDER BY rank ASC");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while( ok && q.next() )
        {
            ok = ( q.value(0).toInt() == rec->timestamps.size() );

            if(ok)
            {
                rec->timestamps.push_back(q.value(1).toDouble());
            }
        }
    }

    // check that the frames exist.

    if(ok)
    {
        for(int i=0; ok && i<rec->num_frames(); i++)
        {
            for(int j=0; ok && j<rec->num_views(); j++)
            {
                ok = rec->directory.exists(rec->getImageFileName(i, j));
            }
        }
    }

    if(ok)
    {
        const int msec = time.elapsed();
        std::cout << "Recording loaded in " << msec << " milliseconds." << std::endl;
    }
    else
    {
        rec.reset();
    }

    return ok;
}

bool Project::removeRecording(int id)
{
    bool ok = true;
    bool has_transaction = false;
    QString dirname;

    // check whether the recording can be removed or not.

    if(ok)
    {
        bool ismutable;
        ok = ( isRecordingMutable(id, ismutable) && ismutable );
    }

    // load name of the directory containing the frames.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT directory FROM recordings WHERE id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            dirname = q.value(0).toString();
            ok = ( dirname.isEmpty() == false );
        }
    }

    // create transaction.

    if(ok)
    {
        ok = mDB.transaction();

        if(ok)
        {
            has_transaction = true;
        }
    }

    // delete recording views.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM recording_views WHERE recording_id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete recording frames.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM recording_frames WHERE recording_id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete recording.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM recordings WHERE id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // commit or rollback transaction.

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    // delete files.

    if(ok)
    {
        QDir recdir = mDir;
        if( recdir.cd(dirname) )
        {
            recdir.removeRecursively();
        }
    }

    recordingModelChanged();

    return ok;
}

bool Project::isRecordingMutable(int id, bool& ismutable)
{
    bool ok = true;

    ismutable = true;

    // check that there is such recording in the databse.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM recordings WHERE id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();
    }

    // check whether some reconstruction is referencing the recording.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(DISTINCT id) FROM reconstructions WHERE recording_id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);

        ok = q.exec() && q.next();

        if(ok)
        {
            ismutable = (q.value(0).toInt() == 0);
        }
    }

    if(ok == false)
    {
        ismutable = false;
    }

    return ok;
}

bool Project::describeRecording(int id, QString& descr)
{
    bool ok = true;

    double duration = 0.0;
    int num_frames = 0;
    int num_views = 0;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT MAX(`rank`)+1 AS `number_of_frames`, MAX(`timestamp`) AS `duration` FROM `recording_frames` WHERE recording_id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
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
        q.prepare("SELECT MAX(`rank`)+1 AS `number_of_views` FROM `recording_views` WHERE `recording_id`=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        if(ok)
        {
            if(q.next())
            {
                num_views = q.value(0).toInt();
            }
            else
            {
                num_views = 0;
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `directory` FROM `recordings` WHERE `id`=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            std::stringstream s;

            s << "<html><head></head><body>" << std::endl;

            s << "<h3>Metadata</h3>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
            s << "<tr><th>name</th><td>" << q.value(0).toString().toHtmlEscaped().toStdString() << "</td></tr>" << std::endl;
            s << "<tr><th>date</th><td>" << q.value(1).toString().toHtmlEscaped().toStdString() << "</td></tr>" << std::endl;
            s << "</table>" << std::endl;

            s << "<h3>Content</h3>" << std::endl;
            s << "<table>" << std::endl;
            s << "<tr><th>Filename:</th><td>" << q.value(2).toString().toHtmlEscaped().toStdString() << "</td></tr>" << std::endl;
            s << "<tr><th>Number of views:</th><td>" << num_views << "</td></tr>" << std::endl;
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
        q.bindValue(0, new_name);
        q.bindValue(1, id);

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
        q.setForwardOnly(true);
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

bool Project::createRecordingDirectory(QDir& dir)
{
    bool ok = false;
    QString relative_path;

    for(int i=0; ok == false && i<1000000; i++)
    {
        relative_path = QString("rec_") + QString::number(i);
        ok = ( mDir.exists(relative_path) == false );
    }

    if(ok)
    {
        dir = mDir;
        ok = mDir.mkdir(relative_path);
    }

    if(ok)
    {
        ok = dir.cd(relative_path);
    }

    if(ok == false)
    {
        dir = QDir();
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
        q.setForwardOnly(true);
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

bool Project::describeReconstruction(int id, QString& descr)
{
    bool ok = isOpen();

    std::string reconstruction_name;
    std::string reconstruction_date;
    std::string recording_name;
    int recording_id = -1;
    std::string rig_name;
    int rig_id = -1;

    int num_frames = 0;
    int num_mappoints = 0;
    int num_densepoints = 0;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "SELECT reconstruction.name, DATETIME(reconstruction.date, 'localtime'), rig.id, rig.name, recording.id, recording.name "
            "FROM reconstructions reconstruction, rigs rig, recordings recording "
            "WHERE rig.id=reconstruction.rig_id AND recording.id=reconstruction.recording_id AND reconstruction.id=?"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            reconstruction_name = q.value(0).toString().toStdString();
            reconstruction_date = q.value(1).toString().toStdString();
            rig_id = q.value(2).toInt();
            recording_id = q.value(4).toInt();
            rig_name = q.value(3).toString().toStdString();
            recording_name = q.value(5).toString().toStdString();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(id) FROM frames WHERE reconstruction_id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            num_frames = q.value(0).toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "SELECT COUNT(DISTINCT projections.mappoint_id) "
            "FROM projections,keypoints,frames "
            "WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            num_mappoints = q.value(0).toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(densepoints.id) FROM densepoints, frames WHERE densepoints.frame_id=frames.id AND frames.reconstruction_id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            num_densepoints = q.value(0).toInt();
        }
    }

    if(ok)
    {
        std::stringstream s;
        s << "<html><head></head><body>" << std::endl;

        s << "<h3>Metadata</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>id</th><td>" << id << "</td></tr>" << std::endl;
        s << "<tr><th>name</th><td>" << htmlEscape(reconstruction_name) << "</td></tr>" << std::endl;
        s << "<tr><th>date</th><td>" << htmlEscape(reconstruction_date) << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "<h3>Input</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>Recording</th><td><em>" << htmlEscape(recording_name) << "</em> (" << recording_id << ")</td></tr>" << std::endl;
        s << "<tr><th>Rig calibration</th><td><em>" << htmlEscape(rig_name) << "</em> (" << rig_id << ")</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "<h3>Reconstruction</h3>" << std::endl;
        s << "<table>" << std::endl;
        s << "<tr><th>Number of frames</th><td>" << num_frames << "</td></tr>" << std::endl;
        s << "<tr><th>Number of mappoints</th><td>" << num_mappoints << "</td></tr>" << std::endl;
        s << "<tr><th>Number of densepoints</th><td>" << num_densepoints << "</td></tr>" << std::endl;
        s << "</table>" << std::endl;

        s << "</body></html>" << std::endl;

        descr = s.str().c_str();
    }

    return ok;
}

bool Project::renameReconstruction(int id, const QString& new_name)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("UPDATE reconstructions SET name=? WHERE id=?");
        q.bindValue(0, new_name);
        q.bindValue(1, id);

        ok = q.exec() && (q.numRowsAffected() >= 1);
    }

    reconstructionModelChanged();

    return ok;
}

bool Project::isReconstructionMutable(int id, bool& ismutable)
{
    bool ok = true;

    ismutable = true;

    // check that there is such reconstruction in the databse.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM reconstructions WHERE id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();
    }

    if(ok)
    {
        ismutable = true;
    }

    if(ok == false)
    {
        ismutable = false;
    }

    return ok;
}

bool Project::removeReconstruction(int id)
{
    bool ok = true;
    bool has_transaction = false;

    // check whether the reconstruction can be removed or not.

    if(ok)
    {
        bool ismutable;
        ok = ( isReconstructionMutable(id, ismutable) && ismutable );
    }

    // create transaction.

    if(ok)
    {
        ok = mDB.transaction();
        if(ok)
        {
            has_transaction = true;
        }
    }

    // delete mappoints.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "DELETE FROM mappoints WHERE id IN ("
            "SELECT projections.mappoint_id FROM projections,keypoints,frames "
            "WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=? )"
        );
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete densepoints.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM densepoints WHERE frame_id IN (SELECT id FROM frames WHERE reconstruction_id=?)");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete projections.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "DELETE FROM projections WHERE id IN ("
            "SELECT projections.id FROM projections, keypoints, frames "
            "WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?)"
        );
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete descriptors

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "DELETE FROM descriptors WHERE keypoint_id IN ("
            "SELECT keypoints.id FROM keypoints, frames WHERE keypoints.frame_id=frames.id AND frames.reconstruction_id=?)"
        );
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete keypoints

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM keypoints WHERE frame_id IN (SELECT id FROM frames WHERE reconstruction_id=?)");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete frame poses.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM poses WHERE id IN (SELECT rig_to_world FROM frames WHERE reconstruction_id=?)");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete frames

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM frames WHERE reconstruction_id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // delete reconstruction

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM reconstructions WHERE id=?");
        q.bindValue(0, id);
        ok = q.exec();
    }

    // commit or rollback transaction.

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    reconstructionModelChanged();

    return ok;
}

bool Project::saveReconstruction(SLAMReconstructionPtr rec, int& id)
{
    std::vector<int> pose_ids;
    std::vector<int> frame_ids;
    std::map<int,int> mappoints; // mappoint id to mappoint db id.

    bool has_transaction = false;
    bool ok = isOpen();

    if(ok)
    {
        ok = ( rec->id < 0 && rec->recording->id >= 0 && rec->calibration->id >= 0 );
    }

    if(ok)
    {
        ok = mDB.transaction();

        if(ok)
        {
            has_transaction = true;
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO reconstructions(name, date, rig_id, recording_id) VALUES(?, DATETIME('now'), ?, ?)");
        q.bindValue(0, rec->name.c_str());
        q.bindValue(1, rec->calibration->id);
        q.bindValue(2, rec->recording->id);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);

        q.prepare("INSERT INTO poses(qx, qy, qz, qw, x, y, z) VALUES(?,?,?,?,?,?,?)");

        pose_ids.resize(rec->frames.size(), -1);

        for(int i=0; ok && i<rec->frames.size(); i++)
        {
            const Sophus::SE3d& pose = rec->frames[i]->frame_to_world;
            const Eigen::Quaterniond r = pose.unit_quaternion();
            const Eigen::Vector3d t = pose.translation();

            q.bindValue(0, r.x());
            q.bindValue(1, r.y());
            q.bindValue(2, r.z());
            q.bindValue(3, r.w());
            q.bindValue(4, t.x());
            q.bindValue(5, t.y());
            q.bindValue(6, t.z());

            ok = q.exec();

            if(ok)
            {
                pose_ids[i] = q.lastInsertId().toInt();
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);

        q.prepare("INSERT INTO frames(reconstruction_id, rank, rank_in_recording, timestamp, rig_to_world, aligned_wrt_previous) VALUES(?,?,?,?,?,?)");

        frame_ids.resize(rec->frames.size(), -1);

        for(int i=0; ok && i<rec->frames.size(); i++)
        {
            SLAMFramePtr frame = rec->frames[i];

            q.bindValue(0, id);
            q.bindValue(1, frame->id);
            q.bindValue(2, frame->rank_in_recording);
            q.bindValue(3, frame->timestamp);
            q.bindValue(4, pose_ids[i]);
            q.bindValue(5, frame->aligned_wrt_previous_frame);
            ok = q.exec();

            if(ok)
            {
                frame_ids[i] = q.lastInsertId().toInt();
            }
        }
    }

    if(ok)
    {
        QSqlQuery q1(mDB);
        q1.prepare("INSERT INTO keypoints (frame_id, view, rank, u, v) VALUES (?,?,?,?,?)");

        QSqlQuery q2(mDB);
        q2.prepare("INSERT INTO projections (keypoint_id, mappoint_id) VALUES(?,?)");

        QSqlQuery q3(mDB);
        q3.prepare("INSERT INTO mappoints(rank, world_x, world_y, world_z) VALUES(?,?,?,?)");

        for(int i=0; ok && i<rec->frames.size(); i++)
        {
            const SLAMFramePtr frame = rec->frames[i];

            for(int v=0; ok && v<2; v++)
            {
                const SLAMView& view = frame->views[v];

                for(int j=0; ok && j<view.keypoints.size(); j++)
                {
                    int keypoint_id = -1;
                    const SLAMMapPointPtr mp = view.tracks[j].mappoint;

                    q1.bindValue(0, frame_ids[i]);
                    q1.bindValue(1, v);
                    q1.bindValue(2, j);
                    q1.bindValue(3, view.keypoints[j].pt.x );
                    q1.bindValue(4, view.keypoints[j].pt.y );
                    ok = q1.exec();

                    if(ok)
                    {
                        keypoint_id = q1.lastInsertId().toInt();
                    }

                    // TODO: save descriptor.

                    if(ok && mp)
                    {
                        int mappoint_id = -1;

                        std::map<int,int>::iterator it = mappoints.find(mp->id);

                        if(mappoints.end() == it)
                        {
                            q3.addBindValue(mp->id);
                            q3.addBindValue(mp->position.x());
                            q3.addBindValue(mp->position.y());
                            q3.addBindValue(mp->position.z());

                            ok = q3.exec();

                            if(ok)
                            {
                                mappoint_id = q3.lastInsertId().toInt();
                                mappoints[mp->id] = mappoint_id;
                            }
                        }
                        else
                        {
                            mappoint_id = it->second;
                        }

                        if(ok)
                        {
                            q2.addBindValue(keypoint_id);
                            q2.addBindValue(mappoint_id);

                            ok = q2.exec();
                        }
                    }
                }
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO densepoints(frame_id, rig_x, rig_y, rig_z, color_red, color_green, color_blue) VALUES (?,?,?,?,?,?,?)");

        for(int i=0; ok && i<rec->frames.size(); i++)
        {
            const SLAMFramePtr frame = rec->frames[i];

            for(int j=0; ok && j<frame->dense_cloud.size(); j++)
            {
                const SLAMColoredPoint& cpt = frame->dense_cloud[j];

                q.bindValue(0, frame_ids[i]);
                q.bindValue(1, cpt.point.x);
                q.bindValue(2, cpt.point.y);
                q.bindValue(3, cpt.point.z);
                q.bindValue(4, cpt.color[2]);
                q.bindValue(5, cpt.color[1]);
                q.bindValue(6, cpt.color[0]);

                ok = q.exec();
            }
        }
    }

    if(has_transaction)
    {
        if(ok)
        {
            mDB.commit();
        }
        else
        {
            mDB.rollback();
        }
    }

    if(ok)
    {
        rec->id = id;
    }
    else
    {
        id = -1;
    }

    reconstructionModelChanged();

    return ok;
}

bool Project::loadReconstruction(int id, SLAMReconstructionPtr& rec)
{
    int recording_id = -1;
    int rig_id = -1;
    std::map<int,SLAMFramePtr> frames;
    std::map<int,SLAMMapPointPtr> mappoints;

    QTime time;
    time.start();

    bool ok = isOpen();

    if(ok)
    {
        rec.reset(new SLAMReconstruction());
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT name,DATETIME(date,'localtime'),rig_id,recording_id FROM reconstructions WHERE id=?");
        q.bindValue(0, id);
        q.setForwardOnly(true);

        ok = q.exec() && q.next();

        if(ok)
        {
            rec->id = id;
            rec->name = q.value(0).toString().toStdString();
            rec->date = q.value(1).toString().toStdString();
            rig_id = q.value(2).toInt();
            recording_id = q.value(3).toInt();
        }
    }

    if(ok)
    {
        ok = loadCalibration(rig_id, rec->calibration);
    }

    if(ok)
    {
        ok = loadRecording(recording_id, rec->recording);
    }

    // load frames.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id, rank, rank_in_recording, timestamp, rig_to_world, aligned_wrt_previous FROM frames WHERE reconstruction_id=? ORDER BY rank ASC");
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        int count = 0;
        while(ok && q.next())
        {
            const int frame_id = q.value(0).toInt();
            SLAMFramePtr frame(new SLAMFrame());
            frames[frame_id] = frame;

            if(ok)
            {
                ok = (q.value(1).toInt() == count);
            }

            if(ok)
            {
                frame->id = count;
                frame->rank_in_recording = q.value(2).toInt();
                frame->timestamp = q.value(3).toDouble();
                frame->aligned_wrt_previous_frame = bool( q.value(5).toInt() );
            }

            if(ok)
            {
                rec->frames.push_back(frame);
            }

            count++;
        }
    }

    // load poses.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "SELECT frames.id, poses.qx, poses.qy, poses.qz, poses.qw, poses.x, poses.y, poses.z "
            "FROM frames, poses "
            "WHERE poses.id=frames.rig_to_world AND frames.reconstruction_id=? ORDER BY frames.rank ASC"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            const int frame_id = q.value(0).toInt();

            std::map<int,SLAMFramePtr>::iterator it = frames.find(frame_id);

            if(it == frames.end())
            {
                ok = false;
            }
            else
            {
                Eigen::Quaterniond r;
                Eigen::Vector3d t;

                r.x() = q.value(1).toDouble();
                r.y() = q.value(2).toDouble();
                r.z() = q.value(3).toDouble();
                r.w() = q.value(4).toDouble();
                t.x() = q.value(5).toDouble();
                t.y() = q.value(6).toDouble();
                t.z() = q.value(7).toDouble();

                Sophus::SE3d& pose = it->second->frame_to_world;
                pose.setQuaternion(r);
                pose.translation() = t;
            }
        }
    }

    // load keypoints.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "SELECT keypoints.frame_id, keypoints.view, keypoints.rank, keypoints.u, keypoints.v "
            "FROM keypoints, frames "
            "WHERE keypoints.frame_id=frames.id AND frames.reconstruction_id=? ORDER BY frames.rank ASC, keypoints.rank ASC"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            SLAMFramePtr frame;

            const int frame_id = q.value(0).toInt();
            const int view = q.value(1).toInt();
            const int rank = q.value(2).toInt();
            const double point_u = q.value(3).toDouble();
            const double point_v = q.value(4).toDouble();

            if(ok)
            {
                ok = (view == 0 || view == 1);
            }

            if(ok)
            {
                std::map<int,SLAMFramePtr>::iterator it = frames.find(frame_id);

                if(it == frames.end())
                {
                    ok = false;
                }
                else
                {
                    frame = it->second;
                }
            }

            if(ok)
            {
                ok = (rank == frame->views[view].keypoints.size());
            }

            if(ok)
            {
                frame->views[view].keypoints.emplace_back();
                frame->views[view].keypoints.back().pt.x = point_u;
                frame->views[view].keypoints.back().pt.y = point_v;
            }
        }
    }

    if(ok)
    {
        for( std::pair<int,SLAMFramePtr> item : frames )
        {
            for( SLAMView& v : item.second->views )
            {
                v.tracks.resize(v.keypoints.size());
            }
        }
    }

    // load mappoints and projections.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare(
            "SELECT keypoints.frame_id, keypoints.view, keypoints.rank, mappoints.id, mappoints.rank, mappoints.world_x, mappoints.world_y, mappoints.world_z "
            "FROM mappoints, keypoints, frames, projections "
            "WHERE projections.mappoint_id=mappoints.id AND projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?"
        );
        q.bindValue(0, id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            SLAMFramePtr frame;
            SLAMMapPointPtr mappoint;

            const int frame_id = q.value(0).toInt();
            const int view = q.value(1).toInt();
            const int keypoint_rank = q.value(2).toInt();
            const int mappoint_id = q.value(3).toInt();
            const int mappoint_rank = q.value(4).toInt();
            const double mappoint_worldx = q.value(5).toDouble();
            const double mappoint_worldy = q.value(6).toDouble();
            const double mappoint_worldz = q.value(7).toDouble();

            if(ok)
            {
                std::map<int,SLAMFramePtr>::iterator it = frames.find(frame_id);

                if(it == frames.end())
                {
                    ok = false;
                }
                else
                {
                    frame = it->second;
                }
            }

            if(ok)
            {
                ok = (view == 0 || view == 1);
            }

            if(ok)
            {
                ok = (0 <= keypoint_rank && keypoint_rank < frame->views[view].keypoints.size());
            }

            if(ok)
            {
                ok = !frame->views[view].tracks[keypoint_rank].mappoint;
            }

            if(ok)
            {
                std::map<int,SLAMMapPointPtr>::iterator it = mappoints.find(mappoint_id);

                if(it == mappoints.end())
                {
                    mappoint.reset(new SLAMMapPoint());
                    mappoints[mappoint_id] = mappoint;

                    mappoint->id = mappoint_rank;
                    mappoint->position.x() = mappoint_worldx;
                    mappoint->position.y() = mappoint_worldy;
                    mappoint->position.z() = mappoint_worldz;
                }
                else
                {
                    mappoint = it->second;
                }
            }

            if(ok)
            {
                frame->views[view].tracks[keypoint_rank].mappoint = mappoint;
            }
        }
    }

    // TODO: load dense points.

    if(ok)
    {
        rec->buildSegments();
        //std::cout << rec->frames.front()->dense_cloud.size() << std::endl;
    }

    if(ok)
    {
        const int msec = time.elapsed();
        std::cout << "Reconstruction loaded in " << msec << " milliseconds." << std::endl;
    }
    else
    {
        rec.reset();
    }

    return ok;
}

std::string Project::htmlEscape(const std::string& from)
{
    return QString(from.c_str()).toHtmlEscaped().toStdString();
}

