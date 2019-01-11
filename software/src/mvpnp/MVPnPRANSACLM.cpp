#include <opencv2/calib3d.hpp>
#include "MVPnPRANSACLM.h"

MVPnP::SolverRANSACLM::SolverRANSACLM()
{
    mInlierThreshold = 6.0;
    mInlierRate = 0.8;
    mSuccessProbability = 0.995;
    mMinimumNumberOfObservations = 10; // minimum number of observations to retrieve models' parameters.
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

