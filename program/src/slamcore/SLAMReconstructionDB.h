#pragma once

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
    std::string getReconstructionName(int id);

    bool loadReconstruction(int id, FramePtr& reconstruction);

    bool saveReconstruction(FramePtr last_frame, const std::string& reconstruction_name);

protected:

    void refreshListOfReconstructions();

    bool savePose(const Sophus::SE3d& pose, int& id);

    bool loadPose(int id, Sophus::SE3d& pose);

    bool saveFrame(int reconstruction_id, FramePtr frame, int& id);

    bool saveView(int frame_id, int rank, View& view, int& id);

protected:

    QSqlDatabase mDB;
    std::vector< std::pair<int,std::string> > mAvailableReconstructions;
};

