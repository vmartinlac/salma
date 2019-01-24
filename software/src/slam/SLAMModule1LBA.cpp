#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include "EdgeProjectP2R.h"
#include "SLAMModule1LBA.h"

SLAMModule1LBA::SLAMModule1LBA(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_LBA, con)
{
}

SLAMModule1LBA::~SLAMModule1LBA()
{
}

bool SLAMModule1LBA::init()
{
    SLAMContextPtr con = context();

    mRig = con->calibration;

    return true;
}

SLAMModuleResult SLAMModule1LBA::operator()()
{
    std::vector<SLAMFramePtr> frames;
    std::vector<SLAMMapPointPtr> mappoints;
    std::map<int,int> projection_count;
    std::map<int,int> mappoint_id_to_local_id;
    SLAMReconstructionPtr reconstr = context()->reconstruction;
    std::vector<g2o::VertexSE3Expmap*> frame_vertices;
    std::vector<g2o::VertexSBAPointXYZ*> mappoint_vertices;
    g2o::SparseOptimizer optimizer;
    bool go_on = true;

    std::cout << "   LOCAL BUNDLE ADJUSTMENT" << std::endl;

    // find which frames are to be vertices of the graph.

    if(go_on)
    {
        const int max_previous_frames = context()->configuration->lba.max_previous_frames;

        int i = static_cast<int>( reconstr->frames.size() ) - 1;
        bool take_next = true;

        while( i >= 0 && frames.size() < max_previous_frames && take_next )
        {
            SLAMFramePtr f = reconstr->frames[i];
            take_next = f->aligned_wrt_previous_frame;
            i--;
            frames.push_back(f);
        }

        std::reverse(frames.begin(), frames.end());

        go_on = (frames.empty() == false);
    }

    // find which mappoints are to be vertices of the graph.

    if(go_on)
    {
        for(SLAMFramePtr frame : frames)
        {
            for(SLAMView& v : frame->views)
            {
                for(int i=0; i<v.keypoints.size(); i++)
                {
                    if(v.tracks[i].mappoint)
                    {
                        const int id = v.tracks[i].mappoint->id;

                        if(projection_count.count(id) == 0)
                        {
                            projection_count[id] = 1;
                        }
                        else
                        {
                            projection_count[id]++;

                            if(projection_count[id] == 2)
                            {
                                mappoints.push_back(v.tracks[i].mappoint);
                            }
                        }
                    }
                }
            }
        }

        // if there are too many mappoints, remove some randomly.
        // TODO: is this a good strategy?

        const int max_mappoints = context()->configuration->lba.max_mappoints;

        while(mappoints.size() > max_mappoints)
        {
            std::uniform_int_distribution<int> unif(0, mappoints.size()-1);

            const int todel = unif(mEngine);

            mappoints[todel].swap(mappoints.back());

            mappoints.pop_back();
        }

        go_on = (mappoints.empty() == false);
    }

    // compute mappoint_id to local_id map.

    if(go_on)
    {
        for(int i=0; i<mappoints.size(); i++)
        {
            mappoint_id_to_local_id[mappoints[i]->id] = i;
        }
    }

    // create vertices and edges.

    if(go_on)
    {
        int next_id = 0;

        for(SLAMFramePtr frame : frames)
        {
            g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();

            if(next_id == 0)
            {
                v->setFixed(true);
            }

            v->setId(next_id);
            next_id++;

            Sophus::SE3d world_to_rig = frame->frame_to_world.inverse();

            v->setEstimate(g2o::SE3Quat(
                world_to_rig.unit_quaternion(),
                world_to_rig.translation() ));

            optimizer.addVertex(v);
            frame_vertices.push_back(v);
        }

        if(frame_vertices.size() != frames.size()) throw std::logic_error("internal error");

        for(SLAMMapPointPtr mp : mappoints)
        {
            g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();

            v->setId(next_id);
            next_id++;

            v->setMarginalized(true);

            v->setEstimate(mp->position);

            optimizer.addVertex(v);
            mappoint_vertices.push_back(v);
        }

        if(mappoint_vertices.size() != mappoints.size()) throw std::logic_error("internal error");

        for(int k=0; k<frames.size(); k++)
        {
            SLAMFramePtr f = frames[k];

            for(int j=0; j<2; j++)
            {
                SLAMView& v = f->views[j];

                for(int i=0; i<v.keypoints.size(); i++)
                {
                    SLAMMapPointPtr mpt = v.tracks[i].mappoint;

                    if(mpt)
                    {
                        std::map<int,int>::iterator it = mappoint_id_to_local_id.find(mpt->id);

                        if(it != mappoint_id_to_local_id.end())
                        {
                            const double sigma_projection = context()->configuration->lba.sigma_projection;

                            const Eigen::Matrix2d information = (1.0/(sigma_projection*sigma_projection)) * Eigen::Matrix2d::Identity();

                            EdgeProjectP2R* e = new EdgeProjectP2R();

                            e->setVertex(0, mappoint_vertices[it->second]);
                            e->setVertex(1, frame_vertices[k]);

                            e->setMeasurement(v.keypoints[i].pt);
                            e->setInformation(information);

                            e->setView(j);
                            e->setRigCalibration(mRig);

                            optimizer.addEdge(e);
                        }
                    }
                }
            }
        }
    }

    // optimize.

    if(go_on)
    {
        std::cout << "      Num frames: " << frames.size() << std::endl;
        std::cout << "      Num mappoints: " << mappoints.size() << std::endl;

        g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>* linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(std::unique_ptr< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType> >(linear_solver));

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3>(block_solver));

        optimizer.setAlgorithm(solver);

        optimizer.setVerbose( context()->configuration->lba.verbose );

        optimizer.initializeOptimization();

        optimizer.optimize( context()->configuration->lba.max_steps );
    }

    // retrieve result.

    if(go_on)
    {
        for(int i=0; i<frames.size(); i++)
        {
            const g2o::SE3Quat se3 = frame_vertices[i]->estimate();
            frames[i]->frame_to_world = Sophus::SE3d( se3.rotation(), se3.translation() ).inverse();
        }

        for(int i=0; i<mappoints.size(); i++)
        {
            mappoints[i]->position = mappoint_vertices[i]->estimate();
        }
    }

    return SLAMModuleResult(false, SLAM_MODULE1_STEREOMATCHER);
}

