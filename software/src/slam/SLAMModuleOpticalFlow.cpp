#include "SLAMModuleOpticalFlow.h"

SLAMModuleOpticalFlow::SLAMModuleOpticalFlow(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleOpticalFlow::~SLAMModuleOpticalFlow()
{
}

bool SLAMModuleOpticalFlow::init()
{
    SLAMContextPtr con = context();

    const int size = con->configuration->opticalflow_window_size;

    const int reference_image_width = con->calibration->cameras[0].calibration->image_size.width;

    const int min_width = 150;
    const int max_level = std::floor( std::log(double(reference_image_width)/double(min_width)) / std::log(2.0) );

    mLKT = cv::SparsePyrLKOpticalFlow::create();
    mLKT->setWinSize(cv::Size(size, size));
    mLKT->setMaxLevel(max_level);

    return true;
}

void SLAMModuleOpticalFlow::operator()()
{
    SLAMReconstructionPtr reconstr = context()->reconstruction;

    const int N = reconstr->frames.size();

    if( N >= 2 )
    {
        for(int i=0; i<2; i++)
        {
            processView( reconstr->frames[N-2]->views[i], reconstr->frames[N-1]->views[i] );
        }
    }
    else if( N == 1 )
    {
        reconstr->frames.back()->frame_to_world = Sophus::SE3d();
        reconstr->frames.back()->aligned_wrt_previous_frame = false;
    }
    else
    {
        throw std::runtime_error("internal error");
    }
}

void SLAMModuleOpticalFlow::processView(const SLAMView& prev_view, SLAMView& curr_view)
{
    const std::vector<SLAMProjection>& proj_prev = prev_view.projections;
    std::vector<SLAMProjection>& proj_curr = curr_view.projections;

    proj_curr.clear();

    if( proj_prev.empty() == false )
    {
        const int N = proj_prev.size();

        std::vector<cv::Point2f> points_prev(N);

        for(int i=0; i<N; i++)
        {
            points_prev[i] = proj_prev[i].point;
        }

        std::vector<cv::Point2f> points_curr;
        std::vector<uint8_t> status;

        if( N > 0 )
        {
            mLKT->calc(
                prev_view.image,
                curr_view.image,
                points_prev,
                points_curr,
                status);
        }


        for(int i=0; i<N; i++)
        {
            if( status[i] > 0 && proj_prev[i].max_lifetime > 0 )
            {
                SLAMProjection proj;
                proj.point = points_curr[i];
                proj.mappoint = proj_prev[i].mappoint;
                proj.max_lifetime = proj_prev[i].max_lifetime - 1;
                proj.type = PROJECTION_TRACKED;

                proj_curr.push_back(proj);
            }
        }
    }
}

