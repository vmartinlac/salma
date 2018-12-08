#include <QVariant>
#include <QSqlError>
#include <QSqlQuery>
#include "SLAMReconstructionDB.h"

SLAMReconstructionDB::SLAMReconstructionDB()
{
}

SLAMReconstructionDB::~SLAMReconstructionDB()
{
    close();
}

bool SLAMReconstructionDB::open(const std::string& path)
{
    bool ok = true;

    mDB = QSqlDatabase::addDatabase("QSQLITE");
    mDB.setDatabaseName(path.c_str());

    ok = mDB.open();

    return ok;
}

void SLAMReconstructionDB::close()
{
    if( mDB.isOpen() )
    {
        mDB.close();
    }
}

int SLAMReconstructionDB::getNumberOfReconstructions()
{
    return mAvailableReconstructions.size();
}

std::string SLAMReconstructionDB::getReconstructionName(int id)
{
    return mAvailableReconstructions.at(id).second;
}

bool SLAMReconstructionDB::saveReconstruction(FramePtr last_frame, const std::string& reconstruction_name)
{
    bool ok = true;
    int reconstruction_id = -1;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO reconstructions(name, reconstruction_date) VALUES(?,DATETIME('NOW'))");
        q.addBindValue(reconstruction_name.c_str());
        ok = q.exec();

        if(ok)
        {
            reconstruction_id = q.lastInsertId().toInt();
        }
        /*
        else
        {
            std::cout << q.lastError().driverText().toStdString() << std::endl;
        }
        */
    }

    if(ok)
    {
        FramePtr f = last_frame;

        while(ok && bool(f))
        {
            int frame_id;
            ok = saveFrame(reconstruction_id, f, frame_id);

            f = f->previous_frame;
        }
    }

    return ok;
}

void SLAMReconstructionDB::refreshListOfReconstructions()
{
    mAvailableReconstructions.clear();

    QSqlQuery q = mDB.exec("SELECT id,name FROM reconstructions");

    while(q.next())
    {
        std::pair<int,std::string> item(q.value(0).toInt(), q.value(1).toString().toStdString());

        mAvailableReconstructions.push_back(item);
    }
}

bool SLAMReconstructionDB::loadPose(int id, Sophus::SE3d& pose)
{
    QSqlQuery q(mDB);
    q.prepare("SELECT qx, qy, qz, qw, x, y, z FROM poses WHERE id=?");
    q.addBindValue(id);
    if(q.exec() && q.next())
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

        return true;
    }
    else
    {
        pose = Sophus::SE3d();
        return false;
    }
}

bool SLAMReconstructionDB::savePose(const Sophus::SE3d& pose, int& id)
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

    if( q.exec() )
    {
        id = q.lastInsertId().toInt();
        return true;
    }
    else
    {
        id = -1;
        return false;
    }
}

bool SLAMReconstructionDB::saveFrame(int reconstruction_id, FramePtr frame, int& id)
{
    bool ok = true;
    int pose_id = -1;

    if(ok)
    {
        ok = savePose(frame->frame_to_world, pose_id);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO frames(reconstruction_id, rank, rig_to_world, aligned_wrt_previous) VALUES(?,?,?,?)");
        q.addBindValue(reconstruction_id);
        q.addBindValue(frame->id);
        q.addBindValue(pose_id);
        q.addBindValue(frame->aligned_wrt_previous_frame);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        int view_id;
        ok = saveView(id, 0, frame->views[0], view_id);
    }

    if(ok)
    {
        int view_id;
        ok = saveView(id, 1, frame->views[1], view_id);
    }

    return ok;
}

bool SLAMReconstructionDB::saveView(int frame_id, int rank, View& view, int& id)
{
    bool ok = true;

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO views(frame_id, rank) VALUES(?,?)");
        q.addBindValue(frame_id);
        q.addBindValue(rank);
        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        // TODO: save projections.
    }

    return ok;
}

