#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <QTime>
#include <QDebug>
#include "SLAMEngineImpl.h"
#include "SLAMPrimitives.h"
#include "FinitePriorityQueue.h"
#include "Tracker.h"

#define SLAM_DEBUG

SLAMEngineImpl::SLAMEngineImpl()
{
    m_feature = cv::ORB::create();
}

void SLAMEngineImpl::run()
{
    std::cout << "=====================" << std::endl;
    std::cout << " SLAM ENGINE STARTED " << std::endl;
    std::cout << "=====================" << std::endl;

    setup();

    m_camera->open();

    while( isInterruptionRequested() == false )
    {
        m_camera->read(m_current_image);

        if(m_current_image.isValid())
        {
            std::cout << "-> Processing frame " << m_frame_id << std::endl;

            switch(m_mode)
            {
            case MODE_INIT:
                std::cout << "Mode is INIT" << std::endl;
                processImageInit();
                break;
            case MODE_SLAM:
                std::cout << "Mode is SLAM" << std::endl;
                processImageSLAM();
                break;
            case MODE_DEAD:
            default:
                std::cout << "Mode is DEAD" << std::endl;
                processImageDead();
                break;
            }

            writeOutput();

            // we assume that m_time_last_frame is not used in INIT mode, so we do not need to set its value before this point.
            m_time_last_frame = m_current_image.getTimestamp();
            m_current_image.moveTo(m_previous_image);
            m_frame_id++;
        }
        else
        {
            QThread::msleep(5);
            //QThread::yieldCurrentThread();
        }
    }

    m_camera->close();

    std::cout << "SLAM ENGINE ENDED" << std::endl;
}

void SLAMEngineImpl::setup()
{
    // TODO
    m_tracking_method = TRACKING_TEMPLATE;
    m_measurement_standard_deviation = 1.8/640.0;
    //

    // check that we have a camera.

    if( !m_camera ) throw std::runtime_error("No camera was set.");

    // setup calibration matrix.

    cv::Mat_<float> K(3,3);
    K <<
        m_parameters.fx, 0.0, m_parameters.cx,
        0.0, m_parameters.fy, m_parameters.cy,
        0.0, 0.0, 1.0;

    // setup lens distortion coefficients.

    cv::Mat_<float> lens_distortion(5, 1);
    lens_distortion <<
        m_parameters.distortion_k1,
        m_parameters.distortion_k2,
        m_parameters.distortion_p1,
        m_parameters.distortion_p2,
        m_parameters.distortion_k3;

    m_calibration_matrix = K;
    m_distortion_coefficients = lens_distortion;

    m_mode = MODE_INIT;
    m_current_image.setValid(false);
    m_time_last_frame = 0.0;
    m_frame_id = 0;

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
    m_camera_state.linear_velocity.setZero();
    m_camera_state.angular_velocity.setZero();
    m_landmarks.clear();
    m_state_covariance.resize(0, 0);
    m_candidate_landmarks.clear();

    m_tracker.setUnitLength( m_parameters.initialization_target_scale );
}

void SLAMEngineImpl::processImageInit()
{
    bool ok;
    cv::Mat pnp_rodrigues;
    cv::Mat pnp_translation;
    
    ok = m_tracker.track( m_current_image.refFrame(), true );

    if( ok )
    {
        ok = ( m_tracker.objectPoints().size() >= m_parameters.min_init_landmarks );
    }

    if( ok )
    {
        ok = cv::solvePnP(
            m_tracker.objectPoints(),
            m_tracker.imagePoints(),
            m_calibration_matrix,
            m_distortion_coefficients,
            pnp_rodrigues,
            pnp_translation,
            false,
            cv::SOLVEPNP_ITERATIVE );
    }

    // initialize m_camera_state.
    if(ok)
    {
        SLAMPrimitives::convertPose(
            pnp_rodrigues, pnp_translation, m_camera_state.attitude, m_camera_state.position);

        m_camera_state.linear_velocity.setZero();
        m_camera_state.angular_velocity.setZero();
    }

    if(ok)
    {
        const std::vector<cv::Point3f>& object_points = m_tracker.objectPoints();
        const std::vector<cv::Point2f>& image_points = m_tracker.imagePoints();

        const int N = object_points.size();

        m_landmarks.clear();
        m_landmarks.reserve( N );

        std::vector<bool> has_descriptor_or_template;
        cv::Mat descriptors;
        std::vector<cv::Mat> templates;

        if( m_tracking_method == TRACKING_DESCRIPTOR)
        {
            // TODO rather use cv::KeyPoint::class_id to store index of keypoint.

            std::vector<cv::KeyPoint> original_keypoints = m_tracker.imageKeyPoints();
            std::vector<cv::KeyPoint> keypoints = original_keypoints;

            m_feature->compute(m_current_image.refFrame(), keypoints, descriptors);

            if( keypoints.size() != original_keypoints.size() ) { throw std::runtime_error("error"); }

            for(int i=0; i<N; i++)
            {
                if( cv::norm( keypoints[i].pt - original_keypoints[i].pt ) > 1.5 )
                {
                    throw std::runtime_error("error");
                }
            }

            has_descriptor_or_template.assign(N, true);
        }
        else if( m_tracking_method == TRACKING_TEMPLATE )
        {
            has_descriptor_or_template.resize(N);
            templates.resize(N);

            cv::Rect viewport(
                m_parameters.patch_size/2+1,
                m_parameters.patch_size/2+1, 
                m_current_image.width() - m_parameters.patch_size/2-1,
                m_current_image.height() - m_parameters.patch_size/2-1 );

            cv::Size size( m_parameters.patch_size, m_parameters.patch_size );

            for(int i=0; i<N; i++)
            {
                if( viewport.contains(image_points[i]) )
                {
                    cv::getRectSubPix( m_current_image.refFrame(), size, image_points[i], templates[i] );
                    has_descriptor_or_template[i] = true;
                }
                else
                {
                    has_descriptor_or_template[i] = false;
                }
            }
        }
        else
        {
            throw std::runtime_error("internal error");
        }

        for(int i=0; i<N; i++)
        {
            if( has_descriptor_or_template[i] )
            {
                m_landmarks.emplace_back();
                Landmark& lm = m_landmarks.back();

                lm.position.x() = object_points[i].x;
                lm.position.y() = object_points[i].y;
                lm.position.z() = object_points[i].z;

                lm.num_failed_detections = 0;
                lm.num_successful_detections = 1;
                lm.last_seen_frame = m_frame_id;

                switch( m_tracking_method )
                {
                case TRACKING_DESCRIPTOR:
                    lm.descriptor_or_template = descriptors.row(i);
                    break;
                case TRACKING_TEMPLATE:
                    lm.descriptor_or_template = templates[i];
                    break;
                default:
                    throw std::runtime_error("internal error");
                }
            }
        }

        const int M = m_landmarks.size();

        if( M >= m_parameters.min_init_landmarks )
        {
            m_state_covariance.resize( 13 + 3*M, 13 + 3*M );

            m_state_covariance.setZero();

            double mean_sigma_landmark = 0.0;

            for(int i=0; i<M; i++)
            {
                Eigen::Vector3d in_camera_frame = m_camera_state.attitude.inverse() * (m_landmarks[i].position - m_camera_state.position);
                const double depth = std::max( m_parameters.min_distance_to_camera, in_camera_frame.z() );
                const double eps = double(m_current_image.width()) * 0.9 / 640.0;
                const double sigma_landmark = eps * depth / std::max(m_parameters.fx, m_parameters.fy);
                m_state_covariance.diagonal().segment(13+3*i, 3).fill(sigma_landmark*sigma_landmark);

                mean_sigma_landmark += sigma_landmark;
            }

            mean_sigma_landmark /= double(M);

            const double sigma_position = 0.8 * mean_sigma_landmark;
            const double sigma_attitude = 0.015; // see misc/initial_state_variance.py.
            const double sigma_linear_velocity = 0.0;
            const double sigma_angular_velocity = 0.0;
            m_state_covariance.diagonal().segment<3>(0).fill(sigma_position*sigma_position);
            m_state_covariance.diagonal().segment<4>(3).fill(sigma_attitude*sigma_attitude);
            m_state_covariance.diagonal().segment<3>(7).fill(sigma_linear_velocity*sigma_linear_velocity);
            m_state_covariance.diagonal().segment<3>(10).fill(sigma_angular_velocity*sigma_angular_velocity);

            m_mode = MODE_SLAM;

            qInfo() << "Successful initialization.";
            qInfo() << "Switching to SLAM mode.";
        }
        else
        {
            m_landmarks.clear();
        }
    }
}

void SLAMEngineImpl::processImageSLAM()
{
    QTime chrono;
    int time_prediction = 0;
    int time_update = 0;

    Eigen::VectorXd mu;
    Eigen::MatrixXd sigma;

    retrieveBelief(mu, sigma);

    chrono.start();
    EKFPredict(mu, sigma);
    time_prediction = chrono.elapsed();

    chrono.start();
    EKFUpdate(mu, sigma);
    time_update = chrono.elapsed();

    storeBelief(mu, sigma);

    std::cout << "prediction : update = " << time_prediction << " : " << time_update << std::endl;

    // TODO: manage candidate landmarks.
}

void SLAMEngineImpl::EKFPredict(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    const double dt = m_current_image.getTimestamp() - m_time_last_frame;

    const int num_landmarks = m_landmarks.size();

    const int dim = 13 + 3*num_landmarks;

    // TODO: setup these constants correctly.
    const double sigma_v = dt*5.3; //dt*0.1;
    const double sigma_w = dt*1.2;

    Eigen::SparseMatrix<double> Q(dim, dim);
    Q.reserve(6);
    Q.insert(7,7) = sigma_v*sigma_v;
    Q.insert(8,8) = sigma_v*sigma_v;
    Q.insert(9,9) = sigma_v*sigma_v;
    Q.insert(10,10) = sigma_w*sigma_w;
    Q.insert(11,11) = sigma_w*sigma_w;
    Q.insert(12,12) = sigma_w*sigma_w;
    Q.makeCompressed();

    Eigen::VectorXd new_mu;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_f(mu, dt, new_mu, J);

    Eigen::MatrixXd new_sigma = J * (sigma + Q) * J.transpose();

    mu.swap(new_mu);
    sigma.swap(new_sigma);
}

void SLAMEngineImpl::EKFUpdate(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    const int num_landmarks = m_landmarks.size();

    const int dim = 13 + 3*num_landmarks;

    const double measurement_sigma = m_measurement_standard_deviation*m_current_image.width();

    const cv::Rect viewport( 0, 0, m_current_image.width(), m_current_image.height() );

    std::vector<int> selection;
    Eigen::VectorXd h;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_h(
        mu,
        m_calibration_matrix,
        m_distortion_coefficients,
        m_parameters.min_distance_to_camera,
        viewport,
        selection,
        h,
        J);

    Eigen::VectorXd residuals;

    computeResiduals(
        mu,
        sigma,
        selection,
        h,
        J,
        residuals);

    const int num_found = selection.size();

    std::cout << "Number of landmarks found in current frame: " << num_found << std::endl;

    if(num_found > 0)
    {
        Eigen::SparseMatrix<double> Q(2*num_found, 2*num_found);
        Q.reserve(2*num_found);

        for(int i=0; i<2*num_found; i++)
        {
            Q.insert(i,i) = measurement_sigma*measurement_sigma;
        }

        Eigen::MatrixXd S = J * sigma * J.transpose() + Q;

        Eigen::LDLT< Eigen::MatrixXd > solver;
        solver.compute( S );

        Eigen::VectorXd new_mu = mu + sigma * J.transpose() * solver.solve( residuals );
        Eigen::MatrixXd new_sigma = sigma - sigma * J.transpose() * solver.solve( J * sigma );

        mu.swap(new_mu);
        sigma.swap(new_sigma);
    }
}

void SLAMEngineImpl::computeResiduals(
    const Eigen::VectorXd& state_mu,
    const Eigen::MatrixXd& state_sigma,
    std::vector<int>& selection,
    Eigen::VectorXd& h,
    Eigen::SparseMatrix<double>& J,
    Eigen::VectorXd& residuals)
{
    const int num_landmarks = m_landmarks.size();

    const int dim = 13 + 3*num_landmarks;

    const int N = selection.size();
    if( J.rows() != 2*N || h.size() != 2*N ) throw std::runtime_error("internal error");

    std::vector<bool> found;

    if( m_tracking_method == TRACKING_TEMPLATE )
    {
        matchWithTemplates( state_mu, state_sigma, selection, h, J, residuals, found);
    }
    else if( m_tracking_method == TRACKING_DESCRIPTOR )
    {
        matchWithDescriptors( state_mu, state_sigma, selection, h, J, residuals, found);
    }
    else
    {
        throw std::runtime_error("internal error");
    }

    if( found.size() != N || residuals.size() != 2*N ) throw std::runtime_error("internal error");

#ifdef SLAM_DEBUG
    {
        cv::Mat debug = m_current_image.refFrame().clone();

        for(int i=0; i<N; i++)
        {
            if( found[i] )
            {
                cv::Point2f measured_pt(h(2*i+0)+residuals(2*i+0), h(2*i+1)+residuals(2*i+1));
                cv::Point2f predicted_pt(h(2*i+0), h(2*i+1));
                // residual = measured - predicted
                cv::line( debug, predicted_pt, measured_pt, cv::Scalar(0,255,0), 2);
            }
        }

        cv::imwrite("debug_output/slam_residuals_"+std::to_string(m_frame_id)+".png", debug);
    }
#endif

    {
        int num_found = 0;
        for(int i=0; i<N; i++)
        {
            if(found[i])
            {
                num_found++;
            }
        }

        Eigen::SparseMatrix<double, Eigen::RowMajor> proj(2*num_found, 2*N);

        std::vector<int> new_selection(N);

        int j = 0;
        for(int i=0; i<N; i++)
        {
            if(found[i])
            {
                if( j >= num_found ) throw std::runtime_error("internal error");

                proj.insert(2*j, 2*i) = 1.0;
                proj.insert(2*j+1, 2*i+1) = 1.0;
                new_selection[j] = selection[i];
                j++;
            }
        }

        if( j != num_found ) throw std::runtime_error("internal error");

        Eigen::SparseMatrix<double> new_J = proj * J;
        Eigen::VectorXd new_residuals = proj * residuals;
        Eigen::VectorXd new_h = proj * h;

        selection.swap(new_selection);
        h.swap(new_h);
        J.swap(new_J);
        residuals.swap(new_residuals);
    }

    /*
    selection.clear();
    h.resize(0);
    J.resize(0, 13+3*num_landmarks);
    residuals.resize(0);
    */
}

void SLAMEngineImpl::matchWithDescriptors(
    const Eigen::VectorXd& state_mu,
    const Eigen::MatrixXd& state_sigma,
    const std::vector<int>& selection,
    const Eigen::VectorXd& h,
    const Eigen::SparseMatrix<double>& J,
    Eigen::VectorXd& residuals,
    std::vector<bool> found)
{
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;
    const int N = selection.size();

    found.assign(N, false);
    residuals.resize(2*N);
    residuals.setZero();

    cv::Mat& image = m_current_image.refFrame();

    cv::Mat mask( image.size(), CV_16S );
    mask = -1;

    const double measurement_sigma = m_measurement_standard_deviation*m_current_image.width();

    Eigen::Matrix2d Q;
    Q <<
        measurement_sigma*measurement_sigma, 0.0,
        0.0, measurement_sigma*measurement_sigma;

    for(int i=0; i<N; i++)
    {
        Eigen::SparseMatrix<double> tmp = J.block(2*i, 0, 2, dim);

        const Eigen::Matrix2d covar = tmp * state_sigma * tmp.transpose() + Q;

        if( std::fabs( covar.determinant() ) > 1.0e-6 )
        {
            const Eigen::Matrix2d covar_inv = covar.inverse();

            int radius = 4 * static_cast<int>( std::sqrt( std::max( covar(0,0), covar(1,1) ) ) );


            // TODO TMP
            radius = std::min(radius, 10*m_current_image.width()/640);
            //

            for(int da=-radius; da<=radius; da++)
            {
                for(int db=-radius; db<=radius; db++)
                {
                    const int a = cvRound(h(2*i+0)) + da;
                    const int b = cvRound(h(2*i+1)) + db;

                    if( 0 <= a && a < image.cols && 0 <= b && b < image.rows )
                    {
                        Eigen::Vector2d delta;
                        delta.x() = double(da);
                        delta.y() = double(db);

                        const double deviation2 = ( delta.transpose() * covar_inv * delta );
                        if( deviation2 >= 0 && deviation2 < 3.0*3.0 )
                        {
                            int16_t& value = mask.at<int16_t>(b, a);

                            if(value == -1)
                            {
                                value = i;
                            }
                            else
                            {
                                value = -2;
                            }
                        }
                    }
                }
            }
        }
    }

/*
#ifdef SLAM_DEBUG
    {
        cv::Mat debug( image.size(), CV_8UC3 );

        for(int i=0; i<debug.cols; i++)
        {
            for(int j=0; j<debug.rows; j++)
            {
                int16_t input = mask.at<int16_t>(j,i);

                cv::Vec3b output;

                if(input == -1)
                {
                    output = cv::Vec3b(0,0,0);
                }
                else if(input == -2)
                {
                    output = cv::Vec3b(255,0,0);
                }
                else
                {
                    int gamma = 255*input/(num_visible-1);
                    output = cv::Vec3b(
                        0,
                        gamma,
                        255-gamma);
                }

                debug.at<cv::Vec3b>(j,i) = output;
            }
        }

        cv::imwrite("debug_output/slam_debug_"+std::to_string(m_frame_id)+"_B.png", debug);
    }
#endif
    */

    std::vector<cv::KeyPoint> corners;
    cv::Mat descriptors;
    m_feature->detectAndCompute(image, cv::Mat(), corners, descriptors);

    std::vector<double> descriptor_distance(N, 0.0);

    for(int i=0; i<corners.size(); i++)
    {
        const int16_t value = mask.at<int16_t>(corners[i].pt);

        if( value >= 0)
        {
            const double dist = cv::norm( descriptors.row(i), m_landmarks[selection[value]].descriptor_or_template );

            if( found[value] == false || dist < descriptor_distance[value] )
            {
                descriptor_distance[value] = dist;
                residuals(2*value+0) = corners[i].pt.x - h(2*value+0);
                residuals(2*value+1) = corners[i].pt.y - h(2*value+1);
                found[value] = true;
            }
        }
    }
}

void SLAMEngineImpl::matchWithTemplates(
    const Eigen::VectorXd& state_mu,
    const Eigen::MatrixXd& state_sigma,
    const std::vector<int>& selection,
    const Eigen::VectorXd& h,
    const Eigen::SparseMatrix<double>& J,
    Eigen::VectorXd& residuals,
    std::vector<bool> found)
{
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;
    const int N = selection.size();

    found.assign(N, false);
    residuals.resize(2*N);
    residuals.setZero();

    cv::Rect image_viewport( 0, 0, m_current_image.width(), m_current_image.height() );

    const double measurement_sigma = m_measurement_standard_deviation*m_current_image.width();

    Eigen::Matrix2d Q;
    Q <<
        measurement_sigma*measurement_sigma, 0.0,
        0.0, measurement_sigma*measurement_sigma;

    for(int i=0; i<N; i++)
    {
        Eigen::SparseMatrix<double> tmp = J.block(2*i, 0, 2, dim);

        Eigen::Matrix2d obs_sigma = tmp * state_sigma * tmp.transpose() + Q;

        const int sx = 1 + static_cast<int>(std::ceil(std::sqrt( obs_sigma(0) ))) + m_parameters.patch_size;
        const int sy = 1 + static_cast<int>(std::ceil(std::sqrt( obs_sigma(1) ))) + m_parameters.patch_size;

        cv::Rect area(
            h(2*i+0) - sx,
            h(2*i+1) - sy,
            2*sx,
            2*sy);

        if( (image_viewport | area) == image_viewport )
        {
            FinitePriorityQueue<int,double> queue(2);

            cv::Mat& landmark_template = m_landmarks[selection[i]].descriptor_or_template;

            cv::Mat result;
            cv::matchTemplate(
                m_current_image.refFrame()(area),
                landmark_template,
                result,
                CV_TM_SQDIFF);

            for(int a=0; a<result.rows; a++)
            {
                for(int b=0; b<result.cols; b++)
                {
                    queue.push(b + a*result.rows, -result.at<float>(a, b));
                }
            }

            const int ind_1st = queue.top();
            const double priority_1st = queue.top_priority();
            queue.pop();
            const int ind_2nd = queue.top();
            const double priority_2nd = queue.top_priority();

            if( (-priority_1st) < 0.5*(-priority_2nd) )
            {
                const int dy = ind_1st % result.rows;
                const int dx = ind_1st / result.rows;

                if( dx < 0 || dy < 0 || dx >= result.cols || dy >= result.rows )
                {
                    throw std::runtime_error("internal error");
                }

                cv::Point pt(
                    area.x + landmark_template.cols/2 + dx,
                    area.y + landmark_template.rows/2 + dy);

                found[i] = true;
                residuals(2*i+0) = pt.x - h(2*i+0);
                residuals(2*i+1) = pt.y - h(2*i+1);
            }
        }
        else
        {
            found[i] == false;
            residuals(2*i+0) = 0.0;
            residuals(2*i+1) = 0.0;
        }
    }
}

void SLAMEngineImpl::retrieveBelief(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    mu.resize(dim);

    mu.head<13>() <<
        m_camera_state.position.x(),
        m_camera_state.position.y(),
        m_camera_state.position.z(),
        m_camera_state.attitude.w(),
        m_camera_state.attitude.x(),
        m_camera_state.attitude.y(),
        m_camera_state.attitude.z(),
        m_camera_state.linear_velocity.x(),
        m_camera_state.linear_velocity.y(),
        m_camera_state.linear_velocity.z(),
        m_camera_state.angular_velocity.x(),
        m_camera_state.angular_velocity.y(),
        m_camera_state.angular_velocity.z();

    for(int i=0; i<num_landmarks; i++)
    {
        mu.segment<3>(13+3*i) = m_landmarks[i].position;
    }

    sigma = m_state_covariance; // we could swap, as m_state_covariance will not be used until next call to storeBelief().
}

void SLAMEngineImpl::storeBelief(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    m_camera_state.position = mu.segment<3>(0);
    m_camera_state.attitude.w() = mu(3);
    m_camera_state.attitude.vec() = mu.segment<3>(4);
    m_camera_state.linear_velocity = mu.segment<3>(7);
    m_camera_state.angular_velocity = mu.segment<3>(10);

    m_camera_state.attitude.normalize();

    const int num_landmarks = m_landmarks.size();

    for(int i=0; i<num_landmarks; i++)
    {
        m_landmarks[i].position = mu.segment<3>(13+3*i);
    }

    m_state_covariance = sigma; // we could swap.
}

void SLAMEngineImpl::processImageDead()
{
    m_output->beginWrite();
    m_output->image = m_current_image.refFrame().clone();
    m_output->mode = "DEAD";
    m_output->endWrite();
    m_output->updated();
}

void SLAMEngineImpl::writeOutput()
{
    cv::Mat output_image = m_current_image.refFrame().clone();

    if( m_mode == MODE_SLAM)
    {
        for(const Landmark& lm : m_landmarks)
        {
            Eigen::Vector3d pos = m_camera_state.attitude.inverse() * (lm.position - m_camera_state.position);

            cv::Point2f pt(
                m_parameters.cx + m_parameters.fx*pos.x()/pos.z(),
                m_parameters.cy + m_parameters.fy*pos.y()/pos.z() );

            const int radius = output_image.cols*5/640;

            cv::Scalar color;
            if( lm.last_seen_frame == m_frame_id )
            {
                color = cv::Scalar(32, 255, 32);
            }
            else
            {
                color = cv::Scalar(255, 32, 32);
            }

            cv::circle(output_image, pt, radius+1, cv::Scalar(0,0,0), -1);
            cv::circle(output_image, pt, radius, color, -1);
        }

        // draw patches.

        /*
        cv::Rect image_rect( 0, 0, output_image.cols, output_image.rows );

        int k = 0;
        for(const Landmark& lm : m_landmarks)
        {
            const int s = m_parameters.patch_size;
            const int mod = output_image.cols/s;
            const int x = (k % mod) * s;
            const int y = (k / mod) * s;

            cv::Rect patch_rect(x, y, lm.patch.cols, lm.patch.rows);

            if( (patch_rect | image_rect) == image_rect )
            {
                output_image(patch_rect) = lm.patch;
            }

            k++;
        }
        */
    }

    std::vector<SLAMOutputLandmark> output_landmarks;
    output_landmarks.reserve( m_landmarks.size() );
    for(Landmark& lm : m_landmarks)
    {
        SLAMOutputLandmark olm;
        olm.position = lm.position;

        output_landmarks.push_back(olm);
    }

    m_output->beginWrite();

    switch( m_mode )
    {
    case MODE_INIT:
        m_output->mode = "INIT";
        break;
    case MODE_SLAM:
        m_output->mode = "SLAM";
        break;
    case MODE_DEAD:
        m_output->mode = "DEAD";
        break;
    default:
        throw std::runtime_error("internal error");
        break;
    }

    m_output->frame_id = m_frame_id;
    m_output->timestamp = m_current_image.getTimestamp();
    m_output->image = output_image;
    m_output->position = m_camera_state.position;
    m_output->attitude = m_camera_state.attitude;
    m_output->linear_velocity = m_camera_state.linear_velocity;
    m_output->angular_velocity = m_camera_state.angular_velocity;
    m_output->landmarks.swap(output_landmarks);

    m_output->endWrite();

    m_output->updated();
}


SLAMEngine* SLAMEngine::create()
{
    return new SLAMEngineImpl();
}

