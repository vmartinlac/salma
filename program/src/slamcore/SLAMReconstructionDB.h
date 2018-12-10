#pragma once

#include <map>
#include <QSqlDatabase>
#include "SLAMDataStructures.h"

class SLAMReconstructionDB
{
public:

    SLAMReconstructionDB();
    ~SLAMReconstructionDB();

    bool open(const std::string& path);
    void close();

    int getNumberOfReconstructions();
    std::string getReconstructionName(int i);

    bool save(const FrameList& frames, const std::string& name);

    bool load(int i, FrameList& last_frame);

protected:

    void refreshListOfReconstructions();

    int mapProjectionTypeToDB(ProjectionType type);

    bool mapProjectionTypeFromDB(int dbtype, ProjectionType& type);

    bool loadReconstruction(int id, FrameList& frames);

    bool saveReconstruction(const FrameList& frames, const std::string& reconstruction_name);

    bool savePose(const Sophus::SE3d& pose, int& id);

    bool loadPose(int id, Sophus::SE3d& pose);

    bool saveFrame(int reconstruction_id, FramePtr frame, int& id);

    bool loadFrame(int id, FramePtr& frame);

    bool saveView(int frame_id, int rank, View& view, int& id);

    bool loadView(int frame_id, int rank, View& view);

    bool saveMapPoint(MapPointPtr mappoint, int& id);

    bool loadMapPoint(int id, MapPointPtr& mappoint);

protected:

    QSqlDatabase mDB;

    //QSqlQuery mStmtInsertProjection;
    //QSqlQuery mStmtSelectReconstructions;
    //QSqlQuery mStmtSelectFrames;

    std::vector< std::pair<int,std::string> > mAvailableReconstructions;
    std::map<int,MapPointPtr> mLoadedMapPoints; // DB it --> mappoint
    std::map<int,int> mSavedMapPoints; // mappoint id --> DB id
};

