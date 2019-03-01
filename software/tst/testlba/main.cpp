#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <random>
#include "SLAMModule1LBA.h"
#include "SLAMDebug.h"
#include "SLAMConfiguration.h"
#include "SLAMContext.h"

int main(int num_args, char** args)
{
    const double sigma_mappoint = 0.1;
    const double sigma_pose_t = 0.1;
    const double sigma_pose_theta = M_PI*0.13;

    std::default_random_engine rng;

    // create camera and rig calibration.

    cv::Mat calibration_matrix = cv::Mat::zeros(3,3,CV_64F);
    calibration_matrix.at<double>(0,0) = 2200.0;
    calibration_matrix.at<double>(0,2) = 650.0;
    calibration_matrix.at<double>(1,1) = 2200.0;
    calibration_matrix.at<double>(1,2) = 460.0;
    calibration_matrix.at<double>(2,2) = 1.0;

    StereoRigCalibrationPtr rig(new StereoRigCalibration());
    rig->cameras[0].calibration_matrix = calibration_matrix;
    rig->cameras[0].camera_to_rig.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);
    rig->cameras[1].calibration_matrix = calibration_matrix;
    rig->cameras[1].camera_to_rig.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);

    // create context.

    SLAMContextPtr con(new SLAMContext());

    con->configuration.reset(new SLAMConfiguration());
    con->calibration = rig;
    con->reconstruction.reset(new SLAMReconstruction());
    con->debug.reset(new SLAMDebug(con->configuration));

    con->configuration->lba.verbose = true;

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

    std::vector<Sophus::SE3d> frame_to_world_ref;

    {
        const int N_frames = 5;

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

            frame_to_world_ref.push_back(frame->frame_to_world);

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
                        rig->cameras[j].calibration_matrix,
                        rig->cameras[j].distortion_coefficients,
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

        for(SLAMMapPointPtr mp : mappoints)
        {
            mp->position.x() += sigma_mappoint * normal(rng);
            mp->position.y() += sigma_mappoint * normal(rng);
            mp->position.z() += sigma_mappoint * normal(rng);
        }
    }

    // add noise to frame positions.

    {
        std::normal_distribution<double> normal;

        bool first = true;

        for(SLAMFramePtr f : con->reconstruction->frames)
        {
            Eigen::Vector3d axis;

            do
            {
                axis.x() = normal(rng);
                axis.y() = normal(rng);
                axis.z() = normal(rng);
            }
            while(axis.norm() < 1.0e-5);

            axis.normalize();

            const double theta = sigma_pose_theta * normal(rng);

            const Eigen::Matrix3d R = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis)).toRotationMatrix();

            if(first)
            {
                first = false;
            }
            else
            {
                f->frame_to_world.translation().x() += sigma_pose_t * normal(rng);
                f->frame_to_world.translation().y() += sigma_pose_t * normal(rng);
                f->frame_to_world.translation().z() += sigma_pose_t * normal(rng);

                const Eigen::Matrix3d new_attitude = f->frame_to_world.rotationMatrix() * R;

                f->frame_to_world.setQuaternion(Eigen::Quaterniond(new_attitude));
            }
        }
    }

    // call LBA module.

    SLAMModulePtr mod(new SLAMModule1LBA(con));

    mod->init();
    (*mod)();

    // check results.

    for(int i=0; i<mappoints.size(); i++)
    {
        const Eigen::Vector3d err = mappoints[i]->position - mappoints_refpos[i];
        std::cout << "[MAPPOINT " << mappoints[i]->id << "] " << err.transpose() << std::endl;
    }

    for(int i=0; i<con->reconstruction->frames.size(); i++)
    {
        SLAMFramePtr f = con->reconstruction->frames[i];
        const Sophus::SE3d ref = frame_to_world_ref[i];
        const Sophus::SE3d opt = f->frame_to_world;
        std::cout << "[FRAME_t " << f->id << "] " << ( opt.translation() - ref.translation() ).transpose() << std::endl;
        std::cout << "[FRAME_q " << f->id << "] " << ( opt.unit_quaternion().coeffs() - ref.unit_quaternion().coeffs() ).transpose() << std::endl;
    }

    return 0;
}

