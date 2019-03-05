#include <QSqlQuery>
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
        ok = ok && q.exec("DELETE FROM `photometric_luts`");
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
        q.setForwardOnly(true);

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

// CALIBRATION

bool Project::saveCalibration(StereoRigCalibrationPtr rig, int& id)
{
    bool has_transaction = false;
    bool ok = true;

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
        q.addBindValue(rig->name.c_str());

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    for(int i=0; ok && i<rig->cameras.size(); i++)
    {
        int camera_id = -1;
        ok = saveCamera(rig->cameras[i], id, i, camera_id);
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

            s << "<h4>Photometric LUT</h4>" << std::endl;
            //s << "<p>Present</p>" << std::endl;
            //
            {
                std::stringstream ss[3];
                for(int i=0; i<256; i++)
                {
                    if(i > 0)
                    {
                        ss[0] << " ; ";
                        ss[1] << " ; ";
                        ss[2] << " ; ";
                    }

                    const cv::Vec3f value = cam.photometric_lut.at<cv::Vec3f>(0, i);

                    ss[0] << value[0];
                    ss[1] << value[1];
                    ss[2] << value[2];
                }

                s << "<ul>" << std::endl;
                s << "<li>Blue: { " << ss[0].str() << " }</li>" << std::endl;
                s << "<li>Green: { " << ss[1].str() << " }</li>" << std::endl;
                s << "<li>Red: { " << ss[2].str() << " }</li>" << std::endl;
                s << "</ul>" << std::endl;
            }
            //

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
    bool ok = isOpen();

    rig.reset(new StereoRigCalibration());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime') FROM `rigs` WHERE `id`=?");

        q.addBindValue(id);
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
        q.prepare("SELECT id,rank_in_rig FROM cameras WHERE rig_id=? ORDER BY rank_in_rig ASC");
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec();

        if(ok)
        {
            int rank = 0;

            while(ok && q.next())
            {
                if(ok)
                {
                    ok = (rank < 2 && q.value(1).toInt() == rank );
                }

                if(ok)
                {
                    ok = loadCamera( q.value(0).toInt(), rig->cameras[rank] );
                }

                rank++;
            }
        }
    }

    if(ok == false)
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
        q.addBindValue(new_name);
        q.addBindValue(id);

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
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete distortion coefficients.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM distortion_coefficients WHERE camera_id IN (SELECT id FROM cameras WHERE rig_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete photometric LUTs.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM photometric_luts WHERE camera_id IN (SELECT id FROM cameras WHERE rig_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete cameras.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM cameras WHERE rig_id=?");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete rig.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM rigs WHERE id=?");
        q.addBindValue(id);
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
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();
    }

    // check whether some reconstruction is referencing the rig.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(DISTINCT id) FROM reconstructions WHERE rig_id=?");
        q.addBindValue(id);
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

bool Project::saveCamera(CameraCalibration& camera, int rig_id, int rank, int& id)
{
    bool ok = isOpen();
    int pose_id = -1;

    if(ok)
    {
        ok = savePose(camera.camera_to_rig, pose_id);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO cameras (rig_id, rank_in_rig, image_width, image_height, fx, fy, cx, cy, distortion_model, camera_to_rig) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0, ?)");
        q.addBindValue(rig_id);
        q.addBindValue(rank);
        q.addBindValue(camera.image_size.width);
        q.addBindValue(camera.image_size.height);
        q.addBindValue(camera.calibration_matrix.at<double>(0,0));
        q.addBindValue(camera.calibration_matrix.at<double>(1,1));
        q.addBindValue(camera.calibration_matrix.at<double>(0,2));
        q.addBindValue(camera.calibration_matrix.at<double>(1,2));
        q.addBindValue(pose_id);

        const bool ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        const cv::Mat lut = camera.photometric_lut;

        if( lut.cols != 256 || lut.rows != 1 || lut.type() != CV_32FC3 ) throw std::runtime_error("internal error");

        for(int j=0; ok && j<256; j++)
        {
            const cv::Vec3f value = lut.at<cv::Vec3f>(0, j);

            QSqlQuery q(mDB);
            q.prepare("INSERT INTO photometric_luts (camera_id, level, red_value, green_value, blue_value) VALUES (?,?,?,?,?)");
            q.addBindValue(id);
            q.addBindValue(j);
            q.addBindValue(value[2]);
            q.addBindValue(value[1]);
            q.addBindValue(value[0]);

            ok = q.exec();
        }
    }

    if(ok)
    {
        const cv::Mat dist = camera.distortion_coefficients;

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
        id = -1;
    }

    return ok;
}

bool Project::loadCamera(int id, CameraCalibration& camera)
{
    bool ok = isOpen();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT image_width, image_height, fx, fy, cx, cy, distortion_model, camera_to_rig FROM cameras WHERE id=?");
        q.addBindValue(id);
        q.setForwardOnly(true);

        bool ok = q.exec() && q.next();

        if(ok)
        {
            ok = ok && (q.value(6).toInt() == 0);
        }

        if(ok)
        {
            ok = loadPose(q.value(7).toInt(), camera.camera_to_rig);
        }

        if(ok)
        {
            camera.image_size.width = q.value(0).toInt();
            camera.image_size.height = q.value(1).toInt();

            camera.calibration_matrix.create(3, 3, CV_64F);
            camera.calibration_matrix.at<double>(0,0) = q.value(2).toDouble();
            camera.calibration_matrix.at<double>(0,1) = 0.0;
            camera.calibration_matrix.at<double>(0,2) = q.value(4).toDouble();
            camera.calibration_matrix.at<double>(1,0) = 0.0;
            camera.calibration_matrix.at<double>(1,1) = q.value(3).toDouble();
            camera.calibration_matrix.at<double>(1,2) = q.value(5).toDouble();
            camera.calibration_matrix.at<double>(2,0) = 0.0;
            camera.calibration_matrix.at<double>(2,1) = 0.0;
            camera.calibration_matrix.at<double>(2,2) = 1.0;
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT level, red_value, green_value, blue_value FROM photometric_luts WHERE camera_id=? ORDER BY level ASC");
        q.addBindValue(id);
        q.setForwardOnly(true);

        ok = q.exec();

        if(ok)
        {
            camera.photometric_lut = cv::Mat::zeros(1, 256, CV_32FC3);

            while(ok && q.next())
            {
                const int level = q.value(0).toInt();
                const float red_value = q.value(1).toFloat();
                const float green_value = q.value(2).toFloat();
                const float blue_value = q.value(3).toFloat();

                if(ok)
                {
                    ok = (0 <= level && level < 256);
                }

                if(ok)
                {
                    camera.photometric_lut.at<cv::Vec3f>(0, level) = cv::Vec3f(blue_value, green_value, red_value);
                }
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `rank`, `value` FROM `distortion_coefficients` WHERE `camera_id`=? ORDER BY `rank` ASC");
        q.addBindValue(id);
        q.setForwardOnly(true);

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
            camera.distortion_coefficients.create(1, values.size(), CV_64F);

            for(size_t j=0; j<values.size(); j++)
            {
                camera.distortion_coefficients.at<double>(0, j) = values[j];
            }
        }
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
        ok = ( rec->id < 0 && rec->frames.size() == rec->num_frames && rec->views.size() == rec->num_frames*rec->num_views && rec->num_frames > 0 );
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
        q.prepare("INSERT INTO recordings(name, date, directory, number_of_views) VALUES(?,DATETIME('now'),?,?)");
        q.addBindValue(rec->name.c_str());
        q.addBindValue(rec->directory.dirName());
        q.addBindValue(rec->num_views);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        for(int i=0; ok && i<rec->num_frames; i++)
        {
            QSqlQuery q(mDB);
            q.prepare("INSERT INTO recording_frames(recording_id, rank, time) VALUES(?,?,?)");
            q.addBindValue(id);
            q.addBindValue(i);
            q.addBindValue(rec->frames[i].timestamp);
            ok = q.exec();

            if(ok)
            {
                const int frame_id = q.lastInsertId().toInt();

                for(int j=0; ok && j<rec->num_views; j++)
                {
                    q.prepare("INSERT INTO recording_views(frame_id,view,filename) VALUES(?,?,?)");
                    q.addBindValue(frame_id);
                    q.addBindValue(j);
                    q.addBindValue(rec->views[i*rec->num_views+j].filename);
                    ok = q.exec();
                }
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

    recordingModelChanged();

    return ok;
}

bool Project::loadRecording(int id, RecordingHeaderPtr& rec)
{
    bool ok = isOpen();

    rec.reset(new RecordingHeader());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `directory`, `number_of_views` FROM `recordings` WHERE `id`=?");
        q.addBindValue(id);
        q.setForwardOnly(true);
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
        q.prepare("SELECT f.rank, f.time, v.view, v.filename FROM recording_views v, recording_frames f WHERE v.frame_id=f.id AND f.recording_id=? ORDER BY f.rank ASC, v.view ASC");
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec();

        rec->num_frames = 0;

        int i = 0;

        while( ok && q.next() )
        {
            const int frame_rank = i / rec->num_views;
            const int view_rank = i % rec->num_views;

            ok = (frame_rank == q.value(0).toInt() && view_rank == q.value(2).toInt());

            if( ok && view_rank == 0 )
            {
                RecordingHeaderFrame f;
                f.timestamp = q.value(1).toDouble();
                rec->frames.push_back(f);
                rec->num_frames++;
            }

            if(ok)
            {
                RecordingHeaderView v;
                v.filename = q.value(3).toString();
                rec->views.push_back(v);
            }

            i++;
        }

        ok = ok && (i == rec->num_views*rec->num_frames && rec->num_frames > 0);
    }

    if(ok == false)
    {
        rec.reset();
    }

    return ok;
}

bool Project::removeRecording(int id)
{
    QString recording_directory_name;
    bool ok = true;
    bool has_transaction = false;

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
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            recording_directory_name = q.value(0).toString();
            ok = ( recording_directory_name.isEmpty() == false );
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
        q.prepare("DELETE FROM recording_views WHERE frame_id IN (SELECT id FROM recording_frames WHERE recording_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete recording frames.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM recording_frames WHERE recording_id=?");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete recording.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM recordings WHERE id=?");
        q.addBindValue(id);
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
        QDir dir = mDir;

        if(dir.cd(recording_directory_name) && dir != mDir)
        {
            // TODO: we do not check return value. Failure to remove files will not be reported to the user.
            dir.removeRecursively();
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
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();
    }

    // check whether some reconstruction is referencing the recording.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT COUNT(DISTINCT id) FROM reconstructions WHERE recording_id=?");
        q.addBindValue(id);
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

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT MAX(`rank`)+1 AS `number_of_frames`, MAX(`time`) AS `duration` FROM recording_frames WHERE recording_id=?");
        q.addBindValue(id);
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
        q.prepare("SELECT `name`, DATETIME(`date`, 'localtime'), `directory`, `number_of_views` FROM `recordings` WHERE `id`=?");
        q.addBindValue(id);
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
            s << "<tr><th>Number of cameras:</th><td>" << q.value(3).toInt() << "</td></tr>" << std::endl;
            s << "<tr><th>Directory:</th><td>" << q.value(2).toString().toHtmlEscaped().toStdString() << "</td></tr>" << std::endl;
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
    QString name;
    bool ok = true;

    if(ok)
    {
        bool go_on = true;

        for(int i=0; go_on && i<1000000; i++)
        {
            name = "rec_" + QString::number(i);
            go_on = mDir.exists(name);
        }

        if(go_on)
        {
            ok = false;
        }
    }

    if(ok)
    {
        dir = mDir;
        ok = ( dir.mkdir(name) && dir.cd(name) );
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
        q.prepare("SELECT reconstruction.name, DATETIME(reconstruction.date, 'localtime'), rig.id, rig.name, recording.id, recording.name FROM reconstructions reconstruction, rigs rig, recordings recording WHERE rig.id=reconstruction.rig_id AND recording.id=reconstruction.recording_id AND reconstruction.id=?");
        q.addBindValue(id);
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
        q.addBindValue(id);
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
        q.prepare("SELECT COUNT(DISTINCT projections.mappoint_id) FROM projections,keypoints,frames WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?");
        q.addBindValue(id);
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
        q.addBindValue(id);
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
        q.addBindValue(new_name);
        q.addBindValue(id);

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
        q.addBindValue(id);
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
        q.prepare("DELETE FROM mappoints WHERE id IN (SELECT projections.mappoint_id FROM projections,keypoints,frames WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete densepoints.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM densepoints WHERE frame_id IN (SELECT id FROM frames WHERE reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete projections.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM projections WHERE id IN (SELECT projections.id FROM projections, keypoints, frames WHERE projections.keypoint_id=keypoints.id AND keypoints.frame_id=frames.id AND frames.reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete descriptors

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM descriptors WHERE keypoint_id IN (SELECT keypoints.id FROM keypoints, frames WHERE keypoints.frame_id=frames.id AND frames.reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete keypoints

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM keypoints WHERE frame_id IN (SELECT id FROM frames WHERE reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete frame poses.

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM poses WHERE id IN (SELECT rig_to_world FROM frames WHERE reconstruction_id=?)");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete frames

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM frames WHERE reconstruction_id=?");
        q.addBindValue(id);
        ok = q.exec();
    }

    // delete reconstruction

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("DELETE FROM reconstructions WHERE id=?");
        q.addBindValue(id);
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
    bool has_transaction = false;
    bool ok = isOpen();

    mMapPointToDB.clear();

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
        q.addBindValue(rec->name.c_str());
        q.addBindValue(rec->calibration->id);
        q.addBindValue(rec->recording->id);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        for(int i=0; ok && i<rec->frames.size(); i++)
        {
            int frame_id = -1;
            ok = saveFrame(rec->frames[i], i, id, frame_id);
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

    mMapPointToDB.clear();

    reconstructionModelChanged();

    return ok;
}

bool Project::saveFrame(SLAMFramePtr frame, int rank, int reconstruction_id, int& id)
{
    int pose_id = -1;

    bool ok = true;

    if(ok)
    {
        ok = savePose(frame->frame_to_world, pose_id);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO frames(reconstruction_id, rank, rank_in_recording, timestamp, rig_to_world, aligned_wrt_previous) VALUES(?,?,?,?,?,?)");
        q.addBindValue(reconstruction_id);
        q.addBindValue(frame->id);
        q.addBindValue(frame->rank_in_recording);
        q.addBindValue(frame->timestamp);
        q.addBindValue(pose_id);
        q.addBindValue(frame->aligned_wrt_previous_frame);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    for(int v=0; ok && v<2; v++)
    {
        const int N_keypoints = frame->views[v].keypoints.size();

        for(int i=0; ok && i<N_keypoints; i++)
        {
            int keypoint_id = -1;

            {
                QSqlQuery q(mDB);
                q.prepare("INSERT INTO keypoints (frame_id, view, rank, u, v) VALUES (?,?,?,?,?)");
                q.addBindValue(id);
                q.addBindValue(v);
                q.addBindValue(i);
                q.addBindValue( frame->views[v].keypoints[i].pt.x );
                q.addBindValue( frame->views[v].keypoints[i].pt.y );
                ok = q.exec();

                if(ok)
                {
                    keypoint_id = q.lastInsertId().toInt();
                }
            }

            if( frame->views[v].tracks[i].mappoint )
            {
                int mappoint_id = -1;

                if(ok)
                {
                    ok = saveMapPoint(frame->views[v].tracks[i].mappoint, mappoint_id);
                }

                if(ok)
                {
                    QSqlQuery q(mDB);
                    q.prepare("INSERT INTO projections (keypoint_id, mappoint_id) VALUES(?,?)");
                    q.addBindValue(keypoint_id);
                    q.addBindValue(mappoint_id);

                    ok = q.exec();
                }
            }

            // TODO: save descriptor.
        }
    }

    if(ok == false)
    {
        id = -1;
    }

    return ok;
}

bool Project::loadMapPoint(int id, SLAMMapPointPtr& mappoint)
{
    bool ok = true;

    std::map<int,SLAMMapPointPtr>::iterator it = mMapPointFromDB.find(id);

    if( it == mMapPointFromDB.end() )
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT rank, world_x, world_y, world_z FROM mappoints WHERE id=?");
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec() && q.next();

        if(ok)
        {
            mappoint.reset(new SLAMMapPoint());
            mappoint->id = q.value(0).toInt();
            mappoint->position.x() = q.value(1).toDouble();
            mappoint->position.y() = q.value(2).toDouble();
            mappoint->position.z() = q.value(3).toDouble();
        }
    }
    else
    {
        mappoint = it->second;
    }

    if(ok == false)
    {
        mappoint.reset();
    }

    return ok;
}

bool Project::saveMapPoint(SLAMMapPointPtr mappoint, int& id)
{
    bool ok = true;

    std::map<int,int>::iterator it = mMapPointToDB.find(mappoint->id);

    if(it == mMapPointToDB.end())
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO mappoints(rank, world_x, world_y, world_z) VALUES(?,?,?,?)");
        q.addBindValue(mappoint->id);
        q.addBindValue(mappoint->position.x());
        q.addBindValue(mappoint->position.y());
        q.addBindValue(mappoint->position.z());

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
            mMapPointToDB[mappoint->id] = id;
        }
    }
    else
    {
        id = it->second;
    }

    if(ok == false)
    {
        id = -1;
    }

    return ok;
}

bool Project::loadReconstruction(int id, SLAMReconstructionPtr& rec)
{
    int recording_id = -1;
    int rig_id = -1;

    bool ok = isOpen();

    rec.reset(new SLAMReconstruction);

    mMapPointFromDB.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT name,DATETIME(date,'localtime'),rig_id,recording_id FROM reconstructions WHERE id=?");
        q.addBindValue(id);
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

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id, rank, rank_in_recording, timestamp, rig_to_world, aligned_wrt_previous FROM frames WHERE reconstruction_id=? ORDER BY rank ASC");
        q.addBindValue(id);
        q.setForwardOnly(true);
        ok = q.exec();

        int count = 0;
        while(ok && q.next())
        {
            const int frame_id = q.value(0).toInt();
            SLAMFramePtr frame(new SLAMFrame());

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
                ok = loadPose( q.value(4).toInt(), frame->frame_to_world );
            }

            if(ok)
            {
                ok = loadKeyPoints(frame_id, frame);
            }

            if(ok)
            {
                rec->frames.push_back(frame);
            }

            count++;
        }
    }

    if(ok)
    {
        rec->buildSegments();
    }

    mMapPointFromDB.clear();

    if(ok == false)
    {
        rec.reset();
    }

    return ok;
}

bool Project::loadKeyPoints(int frame_id, SLAMFramePtr frame)
{
    bool ok = true;

    frame->views[0].keypoints.clear();
    frame->views[0].tracks.clear();
    frame->views[1].keypoints.clear();
    frame->views[1].tracks.clear();

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id, view, rank, u, v FROM keypoints WHERE frame_id=? ORDER BY view ASC, rank ASC");
        q.addBindValue(frame_id);
        q.setForwardOnly(true);
        ok = q.exec();

        while(ok && q.next())
        {
            int keypoint_id = -1;
            int view = -1;

            cv::KeyPoint kpt;

            if(ok)
            {
                keypoint_id = q.value(0).toInt();
                view = q.value(1).toInt();
                ok = (view == 0 || view == 1);
            }

            if(ok)
            {
                ok = ( frame->views[view].keypoints.size() == q.value(2).toInt() );
            }

            if(ok)
            {
                kpt.pt.x = q.value(3).toDouble();
                kpt.pt.y = q.value(4).toDouble();
            }

            if(ok)
            {
                frame->views[view].keypoints.push_back(kpt);
            }
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT projections.mappoint_id, keypoints.view, keypoints.rank FROM projections, keypoints WHERE keypoints.frame_id=? AND projections.keypoint_id=keypoints.id");
        q.addBindValue(frame_id);
        q.setForwardOnly(true);
        ok = q.exec();

        frame->views[0].tracks.resize( frame->views[0].keypoints.size() );
        frame->views[1].tracks.resize( frame->views[1].keypoints.size() );

        while(ok && q.next())
        {
            int keypoint_rank = -1;
            int view = -1;

            if(ok)
            {
                view = q.value(1).toInt();
                ok = (view == 0 || view == 1);
            }

            if(ok)
            {
                keypoint_rank = q.value(2).toInt();
                ok = ( 0 <= keypoint_rank && keypoint_rank < frame->views[view].keypoints.size() );
            }

            if(ok)
            {
                ok = loadMapPoint( q.value(0).toInt(), frame->views[view].tracks[keypoint_rank].mappoint );
            }
        }
    }

    // TODO: load descriptors!

    if(ok == false)
    {
        frame->views[0].keypoints.clear();
        frame->views[0].tracks.clear();
        frame->views[1].keypoints.clear();
        frame->views[1].tracks.clear();
    }

    return ok;
}

std::string Project::htmlEscape(const std::string& from)
{
    return QString(from.c_str()).toHtmlEscaped().toStdString();
}

