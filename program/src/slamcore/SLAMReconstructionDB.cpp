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

bool SLAMReconstructionDB::save(FramePtr last_frame, const std::string& name)
{
    mDB.transaction();

    mSavedMapPoints.clear();

    const bool ret = saveReconstruction(last_frame, name);

    mSavedMapPoints.clear();

    if(ret)
    {
        mDB.commit();
    }

    refreshListOfReconstructions();

    return ret;
}

bool SLAMReconstructionDB::load(int i, FramePtr& last_frame)
{
    mLoadedMapPoints.clear();

    const bool ret = loadReconstruction(mAvailableReconstructions[i].first, last_frame);

    mLoadedMapPoints.clear();

    if( ret == false )
    {
        last_frame.reset();
    }

    return ret;
}

bool SLAMReconstructionDB::loadReconstruction(int id, FramePtr& reconstruction)
{
    bool ok = true;

    reconstruction.reset();

    if(ok)
    {
        // just check that the reconstruction exists.
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM reconstructions WHERE id=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM frames WHERE reconstruction_id=? ORDER BY rank DESC");
        q.addBindValue(id);

        ok = q.exec();

        while(ok && q.next())
        {
            FramePtr newframe;
            ok = loadFrame(id, newframe);

            if(ok)
            {
                ok = bool(newframe);
            }

            if(ok)
            {
                newframe->previous_frame = reconstruction;
                reconstruction = newframe;
            }

            if(ok && bool(newframe->previous_frame))
            {
                ok = ( newframe->previous_frame->id+1 == newframe->id );
            }
        }
    }

    if(ok == false)
    {
        reconstruction.reset();
    }

    return ok;
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
        for( int i=0; ok && i<view.projections.size(); i++)
        {
            Projection& p = view.projections[i];

            int mappoint_id;
            ok = saveMapPoint(p.mappoint, mappoint_id);

            if(ok)
            {
                QSqlQuery q(mDB);
                q.prepare("INSERT INTO projections(view_id, type, u, v, mappoint_id) VALUES(?,?,?,?,?)");
                q.addBindValue(id);
                q.addBindValue(mapProjectionTypeToDB(p.type));
                q.addBindValue(p.point.x);
                q.addBindValue(p.point.y);
                q.addBindValue(mappoint_id);
                ok = q.exec();
            }
        }
    }

    return ok;
}

bool SLAMReconstructionDB::loadFrame(int id, FramePtr& frame)
{
    int pose_id = -1;
    bool ok = true;

    frame.reset(new Frame());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT rank, aligned_wrt_previous, rig_to_world FROM frames WHERE frames.id=?");
        q.addBindValue(id);
        ok = q.exec();

        if(ok)
        {
            frame->id = q.value(0).toInt();
            frame->aligned_wrt_previous_frame = q.value(1).toBool();
            pose_id = q.value(1).toInt();
        }
    }

    if(ok)
    {
        ok = loadPose(pose_id, frame->frame_to_world);
    }

    if(ok)
    {
        ok = loadView(id, 0, frame->views[0]);
    }

    if(ok)
    {
        ok = loadView(id, 1, frame->views[1]);
    }

    if(ok == false)
    {
        frame.reset();
    }

    return false;
}

bool SLAMReconstructionDB::loadView(int frame_id, int rank, View& view)
{
    bool ok = true;
    int view_id;

    view = View();

    if(ok)
    {
        // This query is not necessary.
        // We do it in order to check whether the view exists or not.

        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM views WHERE frame_id=? AND rank=?");
        ok = q.exec() && q.next();
        if(ok)
        {
            view_id = q.value(0).toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT type, u, v, mappoint_id FROM projections WHERE view_id=?");
        q.addBindValue(view_id);
        ok = q.exec();

        if(ok)
        {
            while(ok && q.next())
            {
                Projection p;

                p.point.x = q.value(1).toFloat();
                p.point.y = q.value(2).toFloat();

                ok = loadMapPoint(q.value(3).toInt(), p.mappoint);

                if(ok)
                {
                    ok = mapProjectionTypeFromDB(q.value(0).toInt(), p.type);
                }

                if(ok)
                {
                    view.projections.push_back(p);
                }
            }
        }
    }

    return ok;
}

bool SLAMReconstructionDB::saveMapPoint(MapPointPtr mappoint, int& id)
{
    std::map<int,int>::iterator it = mSavedMapPoints.find(mappoint->id);
    bool ok = bool(mappoint);

    if( it == mSavedMapPoints.end() )
    {
        QSqlQuery q(mDB);

        q.prepare("INSERT INTO mappoints(rank,world_x,world_y,world_z) VALUES(?,?,?,?)");

        q.addBindValue( mappoint->id );
        q.addBindValue( mappoint->position.x() );
        q.addBindValue( mappoint->position.y() );
        q.addBindValue( mappoint->position.z() );

        ok = q.exec();

        if(ok)
        {
            id = q.lastInsertId().toInt();
            mSavedMapPoints[mappoint->id] = id;
        }
    }
    else
    {
        id = it->second;
    }

    return ok;
}

bool SLAMReconstructionDB::loadMapPoint(int id, MapPointPtr& mappoint)
{
    std::map<int,MapPointPtr>::iterator it = mLoadedMapPoints.find(id);
    bool ok = true;

    if( it == mLoadedMapPoints.end() )
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT rank,world_x,world_y,world_z FROM mappoints WHERE id=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            mappoint.reset(new MapPoint());
            mappoint->id = q.value(0).toInt();
            mappoint->position.x() = q.value(1).toFloat();
            mappoint->position.y() = q.value(2).toFloat();
            mappoint->position.z() = q.value(3).toFloat();
            mLoadedMapPoints[id] = mappoint;
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

int SLAMReconstructionDB::mapProjectionTypeToDB(ProjectionType type)
{
    switch(type)
    {
    case PROJECTION_MAPPED:
        return 0;
    case PROJECTION_TRACKED:
        return 1;
    default:
        throw std::runtime_error("incorrect projection type");
    }
}

bool SLAMReconstructionDB::mapProjectionTypeFromDB(int dbtype, ProjectionType& type)
{
    bool ret = true;

    switch(dbtype)
    {
    case 0:
        type = PROJECTION_MAPPED;
        break;
    case 1:
        type = PROJECTION_TRACKED;
        break;
    defult:
        ret = false;
        break;
    }

    return ret;
}

