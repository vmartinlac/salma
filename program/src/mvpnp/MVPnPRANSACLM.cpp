#include <opencv2/calib3d.hpp>
#include "MVPnPRANSACLM.h"

MVPnP::SolverRANSACLM::SolverRANSACLM()
{
    mInlierThreshold = 6.0;
    mInlierRate = 0.8;
    mSuccessProbability = 0.995;
    mMinimumNumberOfObservations = 15; // minimum number of observations to retrieve models' parameters.
}

MVPnP::SolverRANSACLM::~SolverRANSACLM()
{
}

void MVPnP::SolverRANSACLM::setInlierRate(double rate)
{
    mInlierRate = rate;
}

void MVPnP::SolverRANSACLM::setInlierThreshold(double threshold)
{
    mInlierThreshold = threshold;
}

void MVPnP::SolverRANSACLM::setSuccessProbability(double probability)
{
    mSuccessProbability = probability;
}

int MVPnP::SolverRANSACLM::getNumberOfRANSACIterations()
{
    /*
    M nombre d'itérations RANSAC.
    N nombre de points nécessaire pour construire un model.
    t proportion de points bons parmis tous les points.
    alpha proabilité que l'algorithme RANSAC fournisse un bon résultat.

    1 - ( 1 - t^N )^M > alpha

    1 - alpha > (1 - t^N)^M
    log( 1 - alpha ) > M log( 1 - t^N )
    M = ceil[ log(1 - alpha) / log(1 - t^N) ]
    */

    return (int) std::ceil( log(1.0 - mSuccessProbability) / log(1.0 - std::pow(mInlierRate, double(mMinimumNumberOfObservations))) );
}

bool MVPnP::SolverRANSACLM::run(const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers)
{
    bool ret = false;

    if(use_ransac)
    {
        buildWholeSampleArray(views);

        if( mWholeSample.size() >= mMinimumNumberOfObservations*3/2 )
        {
            bool first = true;

            std::vector< std::vector<bool> > unused;

            Sophus::SE3d best_rig_to_world;
            int best_score = 0;
            std::vector< std::vector<bool> > best_inliers;

            const int number_of_iterations = getNumberOfRANSACIterations();

            for(int round=0; round<number_of_iterations; round++)
            {
                std::vector<View> sample;
                extractSample(views, mMinimumNumberOfObservations, sample);

                Sophus::SE3d candidate_rig_to_world = rig_to_world;

                if( SolverLM::run(sample, candidate_rig_to_world, false, unused) )
                {
                    std::vector< std::vector<bool> > candidate_inliers;
                    const int candidate_score = findInliers(views, candidate_rig_to_world, mInlierThreshold, candidate_inliers);
                    //std::cout << "Number of inliers: " << candidate_score << std::endl;

                    if( first || candidate_score > best_score )
                    {
                        best_score = candidate_score;
                        best_rig_to_world = std::move(candidate_rig_to_world);
                        best_inliers.swap(candidate_inliers);
                        first = false;
                    }
                }
            }

            if( first == false && best_score >= mMinimumNumberOfObservations )
            {
                std::vector<View> views_made_of_inliers;
                extractSelection(views, best_inliers, views_made_of_inliers);

                ret = SolverLM::run(views_made_of_inliers, best_rig_to_world, false, unused);

                if(ret)
                {
                    rig_to_world = best_rig_to_world;
                    findInliers( views, best_rig_to_world, mInlierThreshold, inliers);
                }
            }
        }
    }
    else
    {
        ret = SolverLM::run(views, rig_to_world, false, inliers);
    }

    return ret;
}

int MVPnP::SolverRANSACLM::extractSelection(
    const std::vector<View>& views,
    std::vector< std::vector<bool> >& selection,
    std::vector<View>& new_views)
{
    if( selection.size() != views.size() ) throw std::runtime_error("internal error");

    int count;

    new_views.resize(views.size());

    for(int i=0; i<views.size(); i++)
    {
        if( views[i].points.size() != selection[i].size() ) throw std::runtime_error("internal error");

        new_views[i].calibration_matrix = views[i].calibration_matrix;
        new_views[i].distortion_coefficients = views[i].distortion_coefficients;
        new_views[i].rig_to_camera = views[i].rig_to_camera;

        for(int j=0; j<views[i].points.size(); j++)
        {
            if( selection[i][j] )
            {
                new_views[i].points.push_back( views[i].points[j] );
                new_views[i].projections.push_back( views[i].projections[j] );
                count++;
            }
        }
    }

    return count;
}

int MVPnP::SolverRANSACLM::findInliers(
    const std::vector<View>& views,
    const Sophus::SE3d& rig_to_world,
    double threshold,
    std::vector< std::vector<bool> >& inliers)
{
    int inlier_count = 0;

    inliers.clear();
    inliers.resize( views.size() );

    for(int i=0; i<views.size(); i++)
    {
        const int N = views[i].points.size();

        std::vector<cv::Point3f> to_project(N);

        const Sophus::SE3d world_to_camera = views[i].rig_to_camera * rig_to_world.inverse();

        for(int j=0; j<N; j++)
        {
            Eigen::Vector3d in_world_frame;
            in_world_frame.x() = views[i].points[j].x;
            in_world_frame.y() = views[i].points[j].y;
            in_world_frame.z() = views[i].points[j].z;

            Eigen::Vector3d in_camera_frame = world_to_camera * in_world_frame;

            to_project[j].x = in_camera_frame.x();
            to_project[j].y = in_camera_frame.y();
            to_project[j].z = in_camera_frame.z();
        }

        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            to_project,
            cv::Mat::zeros(3,1,CV_64F),
            cv::Mat::zeros(3,1,CV_64F),
            views[i].calibration_matrix,
            views[i].distortion_coefficients,
            projected);

        if(projected.size() != N) throw std::runtime_error("internal error");

        inliers[i].resize(N);

        for(int j=0; j<N; j++)
        {
            const double distance = cv::norm(projected[j] - views[i].projections[j]);

            inliers[i][j] = (distance < threshold);

            if( inliers[i][j] )
            {
                inlier_count++;
            }
        }
    }

    return inlier_count;
}

void MVPnP::SolverRANSACLM::buildWholeSampleArray(const std::vector<View>& views)
{
    mWholeSample.clear();

    for(int i=0; i<views.size(); i++)
    {
        for(int j=0; j<views[i].points.size(); j++)
        {
            mWholeSample.push_back( Sample(i,j) );
        }
    }
}

void MVPnP::SolverRANSACLM::extractSample( const std::vector<View>& views, int maxsize, std::vector<View>& subviews )
{
    subviews.clear();
    subviews.resize(views.size());

    for(int i=0; i<views.size(); i++)
    {
        subviews[i].calibration_matrix = views[i].calibration_matrix;
        subviews[i].distortion_coefficients = views[i].distortion_coefficients;
        subviews[i].rig_to_camera = views[i].rig_to_camera;
    }

    for( int i=0; i < maxsize && i < mWholeSample.size(); i++ )
    {
        std::uniform_int_distribution<int> distrib(i, mWholeSample.size()-1);

        const int j = distrib(mEngine);

        if( i != j )
        {
            std::swap(mWholeSample[i], mWholeSample[j]);
        }

        Sample& s = mWholeSample[i];

        subviews[s.view].points.push_back( views[s.view].points[s.point] );
        subviews[s.view].projections.push_back( views[s.view].projections[s.point] );
    }
}

