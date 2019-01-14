#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModuleLBA.h"
#include "SLAMDebug.h"
#include "SLAMConfiguration.h"
#include "SLAMContext.h"

int main(int num_args, char** args)
{
    std::default_random_engine rng;

    // create camera and rig calibration.

    CameraCalibrationDataPtr camera(new CameraCalibrationData());
    camera->calibration_matrix = cv::Mat::zeros(3,3,CV_64F);
    camera->calibration_matrix.at<double>(0,0) = 2200.0;
    camera->calibration_matrix.at<double>(0,2) = 650.0;
    camera->calibration_matrix.at<double>(1,1) = 2200.0;
    camera->calibration_matrix.at<double>(1,2) = 460.0;
    camera->calibration_matrix.at<double>(2,2) = 1.0;

    StereoRigCalibrationDataPtr rig(new StereoRigCalibrationData());
    rig->cameras[0].calibration = camera;
    rig->cameras[0].camera_to_rig.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
    rig->cameras[1].calibration = camera;
    rig->cameras[1].camera_to_rig.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);

    // create context.

    SLAMContextPtr con(new SLAMContext());

    con->configuration.reset(new SLAMConfiguration());
    con->calibration = rig;
    con->reconstruction.reset(new SLAMReconstruction());
    con->debug.reset(new SLAMDebug(con->configuration));

    // create mappoints.

    std::vector<SLAMMapPointPtr> mappoints;
    std::vector<Eigen::Vector3d> mappoints_refpos;

    {
        const int NX = 8;
        const int NY = 8;
        const double lx = 5.0;
        const double ly = 5.0;
        const double z = 20.0;

        int count = 0;

        for(int i=0; i<NX; i++)
        {
            for(int j=0; j<NY; j++)
            {
                SLAMMapPointPtr mp(new SLAMMapPoint());

                mp->id = count++;
                mp->position.x() = lx*( double(i)/double(NX-1) - 0.5 );
                mp->position.y() = ly*( double(j)/double(NY-1) - 0.5 );
                mp->position.z() = z;

                mappoints.push_back(mp);
                mappoints_refpos.push_back( mp->position );
            }
        }
    }

    // create frames.

    {
        const int N_frames = 4;

        for(int i=0; i<N_frames; i++)
        {
            const double z = double(i)*0.8;

            SLAMFramePtr frame(new SLAMFrame());
            con->reconstruction->frames.push_back(frame);

            frame->id = i;
            frame->rank_in_recording = i;
            frame->timestamp = double(i)*1.0/30.0;
            frame->aligned_wrt_previous_frame = (i > 0);
            frame->frame_to_world = Sophus::SE3d( Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, z) );

            for(SLAMMapPointPtr mp : mappoints)
            {
                for(int j=0; j<2; j++)
                {
                    const Eigen::Vector3d in_camera = rig->cameras[j].camera_to_rig.inverse() * frame->frame_to_world.inverse() * mp->position;

                    const std::vector<cv::Point3f> to_project{ cv::Point3f(in_camera.x(), in_camera.y(), in_camera.z()) };

                    std::vector<cv::Point2f> projected;

                    cv::projectPoints(
                        to_project,
                        cv::Mat::zeros(3,1,CV_64F),
                        cv::Mat::zeros(3,1,CV_64F),
                        rig->cameras[j].calibration->calibration_matrix,
                        rig->cameras[j].calibration->distortion_coefficients,
                        projected);

                    cv::KeyPoint kpt;
                    kpt.pt = projected.front();

                    SLAMTrack track;
                    track.mappoint = mp;

                    frame->views[j].keypoints.push_back(kpt);
                    frame->views[j].tracks.push_back(track);
                }
            }
        }
    }

    // add noise to mappoints.

    {
        std::normal_distribution<double> normal;

        const double sigma = 0.1;

        for(SLAMMapPointPtr mp : mappoints)
        {
            mp->position.x() += sigma * normal(rng);
            mp->position.y() += sigma * normal(rng);
            mp->position.z() += sigma * normal(rng);
        }
    }

    // call LBA module.

    SLAMModulePtr mod(new SLAMModuleLBA(con));

    mod->init();
    (*mod)();

    // check results.

    for(int i=0; i<mappoints.size(); i++)
    {
        const Eigen::Vector3d err = mappoints[i]->position - mappoints_refpos[i];
        std::cout << err.transpose() << std::endl;
    }

    return 0;
}

