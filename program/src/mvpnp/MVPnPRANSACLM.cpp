#include "MVPnPRANSACLM.h"

MVPnP::SolverRANSACLM::SolverRANSACLM()
{
}

MVPnP::SolverRANSACLM::~SolverRANSACLM()
{
}

bool MVPnP::SolverRANSACLM::run(const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers)
{
    bool ret = false;

    if(use_ransac)
    {
        buildWholeSampleArray(views);

        if( mWholeSample.size() >= 20 )
        {
            int num_rounds = 100;

            while(num_rounds > 0)
            {
                num_rounds--;

                std::vector<View> subviews;
                extractSample(views, 15, subviews);

                if( SolverLM::run(subviews, rig_to_world, false, inliers) )
                {
                    ;
        // TODO: finish this function.
                }
            }
        }
        else
        {
            ret = false;
        }
    }
    else
    {
        ret = SolverLM::run(views, rig_to_world, false, inliers);
    }

    return ret;
}

int MVPnP::SolverRANSACLM::findInliers(
    const std::vector<View>& views,
    const Sophus::SE3d& rig_to_world,
    double threshold,
    std::vector< std::vector<bool> >& inliers)
{
    int inlier_count = 0;

    inliers.resize( views.size() );

    for(int i=0; i<views.size(); i++)
    {
        inliers.resize(views[i].points.size());

        // TODO: finish this function.
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

    std::shuffle(mWholeSample.begin(), mWholeSample.end(), mEngine);
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

