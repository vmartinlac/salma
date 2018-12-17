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

    if(ok)
    {
        mDB = QSqlDatabase::database();
        ok = mDB.isValid();
    }

    if(ok)
    {
        mDB.setDatabaseName(path.c_str());
        ok = mDB.open();
    }

    if(ok)
    {
        refreshListOfReconstructions();
    }

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

bool SLAMReconstructionDB::save(ReconstructionPtr rec, const std::string& name)
{
    mDB.transaction();

    mSavedMapPoints.clear();

    const bool ret = saveReconstruction(rec, name);

    mSavedMapPoints.clear();

    if(ret)
    {
        mDB.commit();
    }

    refreshListOfReconstructions();

    return ret;
}

bool SLAMReconstructionDB::load(int i, ReconstructionPtr& rec)
{
    mLoadedMapPoints.clear();

    const bool ret = loadReconstruction(mAvailableReconstructions.at(i).first, rec);

    mLoadedMapPoints.clear();

    if( ret == false )
    {
        rec.reset();
    }

    return ret;
}

bool SLAMReconstructionDB::loadReconstruction(int id, ReconstructionPtr& rec)
{
    int left_camera_to_rig = -1;
    int right_camera_to_rig = -1;
    bool ok = true;

    rec.reset(new Reconstruction());

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT left_camera_to_rig, right_camera_to_rig FROM rigs, reconstructions WHERE reconstructions.rig_id = rigs.id AND reconstructions.id=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            left_camera_to_rig = q.value(0).toInt();
            right_camera_to_rig = q.value(1).toInt();
        }
    }

    if(ok)
    {
        ok = loadPose(left_camera_to_rig, rec->left_camera_to_rig);
    }

    if(ok)
    {
        ok = loadPose(right_camera_to_rig, rec->right_camera_to_rig);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("SELECT id FROM frames WHERE reconstruction_id=? ORDER BY rank DESC");
        q.addBindValue(id);

        ok = q.exec();

        FramePtr prevframe;

        while(ok && q.next())
        {
            FramePtr newframe;

            const int frame_id = q.value(0).toInt();

            ok = loadFrame(frame_id, newframe);

            if(ok)
            {
                ok = bool(newframe);
            }

            if(ok)
            {
                rec->frames.push_front(newframe);
            }

            if(ok && bool(prevframe))
            {
                ok = ( prevframe->id == newframe->id+1 );
            }

            prevframe = newframe;
        }
    }

    if(ok == false)
    {
        rec.reset();
    }

    return ok;
}

bool SLAMReconstructionDB::saveReconstruction(ReconstructionPtr rec, const std::string& reconstruction_name)
{
    bool ok = true;
    int reconstruction_id = -1;
    int left_pose_id = -1;
    int right_pose_id = -1;
    int rig_id = -1;

    if(ok)
    {
        ok = savePose(rec->left_camera_to_rig, left_pose_id);
    }

    if(ok)
    {
        ok = savePose(rec->right_camera_to_rig, right_pose_id);
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO rigs(left_camera_to_rig, right_camera_to_rig) VALUES(?,?)");
        q.addBindValue(left_pose_id);
        q.addBindValue(right_pose_id);
        ok = q.exec();

        if(ok)
        {
            rig_id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        QSqlQuery q(mDB);
        q.prepare("INSERT INTO reconstructions(name, reconstruction_date, rig_id) VALUES(?,DATETIME('NOW'),?)");
        q.addBindValue(reconstruction_name.c_str());
        q.addBindValue(rig_id);
        ok = q.exec();

        if(ok)
        {
            reconstruction_id = q.lastInsertId().toInt();
        }
    }

    if(ok)
    {
        FrameList::const_iterator it = rec->frames.begin();

        while(ok && it != rec->frames.end())
        {
            int frame_id;
            ok = saveFrame(reconstruction_id, *it, frame_id);

            it++;
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

    const bool ok = ( q.exec() && q.next() );

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
        q.prepare("SELECT rank, aligned_wrt_previous, rig_to_world FROM frames WHERE id=?");
        q.addBindValue(id);
        ok = q.exec() && q.next();

        if(ok)
        {
            frame->id = q.value(0).toInt();
            frame->aligned_wrt_previous_frame = q.value(1).toBool();
            pose_id = q.value(2).toInt();
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

    return ok;
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
        q.addBindValue(frame_id);
        q.addBindValue(rank);
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

