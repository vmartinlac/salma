#include "CameraCalibrationOperation.h"

CameraCalibrationOperation::CameraCalibrationOperation()
{
}

CameraCalibrationOperation::~CameraCalibrationOperation()
{
}

Operation::OperationName CameraCalibrationOperation::getOperation()
{
    return OPERATION_CAMERA_CALIBRATION;
}

CameraCalibrationOperation* CameraCalibrationOperation::cameraCalibration()
{
    return this;
}

void CameraCalibrationOperation::before()
{
}

bool CameraCalibrationOperation::step()
{
    return false;
}

void CameraCalibrationOperation::after()
{
}

/*
#include <QTime>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include "CameraCalibrationThread.h"
#include "Tracker.h"
#include "CameraCalibrationData.h"

CameraCalibrationThread::CameraCalibrationThread(
    CameraCalibrationParameters* parameters,
    VideoInputPort* video,
    CameraCalibrationStatsInputPort* stats,
    QObject* parent) : QThread(parent)
{
    mParameters = parameters;
    mVideo = video;
    mStats = stats;
    mNumFrames = 0;
}

CameraCalibrationThread::~CameraCalibrationThread()
{
    if(isRunning())
    {
        requestInterruption();
        wait();
    }
}

void CameraCalibrationThread::run()
{
    // number of successful track needed before an attempt to calibrate.
    const int num_successful_trackings_needed = 40;

    // tracker which tries to generate object/image correspondances from a view of the target.
    target::Tracker tracker;

    // used to add some delay between a successful tracking and the next attempt to track the target.
    bool at_least_one_successful_tracking = false;
    QTime clock;
    const int milliseconds_between_tracking = 700;

    // retrieve parameters data.
    CameraCalibrationParametersData params;
    mParameters->read(params);

    // init output stats.
    mStats->beginWrite();
    mStats->data().camera_name = params.camera->getHumanName();
    mStats->data().output_path = params.output_path;
    mStats->data().frame_count = 0;
    mStats->data().attempted_tracking_count = 0;
    mStats->data().successful_tracking_count = 0;
    mStats->endWrite();
    int frame_count = 0;
    int successful_tracking_count = 0;
    int attempted_tracking_count = 0;

    // where to store 
    bool can_calibrate = false;
    std::vector< std::vector<cv::Point3f> > object_points;
    std::vector< std::vector<cv::Point2f> > image_points;
    cv::Size image_size;

    // check that output file can be open.

    bool lookslikeoutputfileisok = false;

    {
        std::ofstream outputfile(params.output_path.c_str(), std::ofstream::out);
        lookslikeoutputfileisok = outputfile.is_open();
        if(lookslikeoutputfileisok)
        {
            outputfile.close();
        }
        else
        {
            std::cout << "Could not open output file!" << std::endl;
        }
    }

    if( params.camera && lookslikeoutputfileisok )
    {
        params.camera->open();

        while( isInterruptionRequested() == false && can_calibrate == false )
        {
            Image image;
            params.camera->read(image);

            if(image.isValid())
            {
                if( at_least_one_successful_tracking == false || clock.elapsed() > milliseconds_between_tracking )
                {
                    const int target_found = tracker.track(image.refFrame(), false);

                    if( target_found )
                    {
                        image_points.push_back( tracker.imagePoints() );
                        object_points.push_back( tracker.objectPoints() );

                        successful_tracking_count++;

                        clock.start();
                        at_least_one_successful_tracking = true;

                        image_size = image.refFrame().size();
                        can_calibrate = (object_points.size() >= num_successful_trackings_needed );
                    }

                    attempted_tracking_count++;
                }

                frame_count++;

                mVideo->beginWrite();
                mVideo->data().image = image.refFrame();
                mVideo->endWrite();

                mStats->beginWrite();
                mStats->data().frame_count = frame_count;
                mStats->data().successful_tracking_count = successful_tracking_count;
                mStats->data().attempted_tracking_count = attempted_tracking_count;
                mStats->endWrite();
            }
        }

        params.camera->close();

        if( can_calibrate )
        {
            CameraCalibrationData calibration;

            calibration.image_size = image_size;

            std::vector<cv::Mat> rotations;
            std::vector<cv::Mat> translations;

            const double err = cv::calibrateCamera(
                object_points,
                image_points,
                image_size,
                calibration.calibration_matrix,
                calibration.distortion_coefficients,
                rotations,
                translations);
            
            std::cout << "Reprojection error: " << err << std::endl;

            const bool save_ret = calibration.saveToFile(params.output_path);

            if(save_ret == false)
            {
                std::cout << "Could not save calibration data to file!" << std::endl;
            }
        }
    }
}

*/
