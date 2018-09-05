#include "SLAMPrimitives.h"

void SLAMPrimitives::compute_f(
    const Eigen::VectorXd& X,
    double dt,
    Eigen::VectorXd& f,
    Eigen::SparseMatrix<double>& J)
{
    const int dim = X.size();

    if( ( dim - 13) % 3 != 0 ) throw std::runtime_error("internal error");

    const int num_landmarks = (dim - 13)/3;

    const double x1 = X(0);
    const double x2 = X(1);
    const double x3 = X(2);

    const double a0 = X(3);
    const double a1 = X(4);
    const double a2 = X(5);
    const double a3 = X(6);

    const double v1 = X(7);
    const double v2 = X(8);
    const double v3 = X(9);

    double w1 = X(10);
    double w2 = X(11);
    double w3 = X(12);

    // TODO: handle small rotations in a better way.

    double norm_w = std::sqrt(w1*w1 + w2*w2 + w3*w3);
    double axis1;
    double axis2;
    double axis3;
    const double theta = 0.5*norm_w*dt;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    double sin_theta_over_norm_w;

    if( norm_w < 1.0e-10 )
    {
        norm_w = 0.0;
        axis1 = 1.0;
        axis2 = 0.0;
        axis3 = 0.0;
        sin_theta_over_norm_w = 0.0;
    }
    else
    {
        axis1 = w1/norm_w;
        axis2 = w2/norm_w;
        axis3 = w3/norm_w;
        sin_theta_over_norm_w = sin_theta / norm_w;
    }

    const double r0 = cos_theta;
    const double r1 = sin_theta * axis1;
    const double r2 = sin_theta * axis2;
    const double r3 = sin_theta * axis3;

    // fill f.

    f.resize(dim);

    f.head<13>() <<
        x1 + dt*v1,
        x2 + dt*v2,
        x3 + dt*v3,
        a0*r0 - a1*r1 - a2*r2 - a3*r3,
        a0*r1 + r0*a1 + (a2*r3 - a3*r2),
        a0*r2 + r0*a2 + (a3*r1 - a1*r3),
        a0*r3 + r0*a3 + (a1*r2 - a2*r1),
        v1,
        v2,
        v3,
        w1,
        w2,
        w3;

    //f.segment<4>(3).normalize();

    f.tail(dim-13) = X.tail(dim-13);

    // fill J.

    Eigen::SparseMatrix<double, Eigen::RowMajor> Jrm;

    Jrm.resize(dim, dim);
    Jrm.setZero();
    Jrm.reserve(40 + 3*num_landmarks);

    // position.

    Jrm.insert(0,0) = 1.0;
    Jrm.insert(0,7) = dt;
    Jrm.insert(1,1) = 1.0;
    Jrm.insert(1,8) = dt;
    Jrm.insert(2,2) = 1.0;
    Jrm.insert(2,9) = dt;

    // attitude.

    // Generated automatically by python script system2.py.
    // BEGIN
    Jrm.insert(3,3) = r0;
    Jrm.insert(3,4) = -r1;
    Jrm.insert(3,5) = -r2;
    Jrm.insert(3,6) = -r3;
    Jrm.insert(3,10) = -0.5*a0*dt*r1 - 0.5*a1*dt*axis1*axis1*cos_theta + 1.0*a1*axis1*axis1*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis3*cos_theta + 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,11) = -0.5*a0*dt*r2 - 0.5*a1*dt*axis1*axis2*cos_theta + 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis2*cos_theta + 1.0*a2*axis2*axis2*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,12) = -0.5*a0*dt*r3 - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis3*cos_theta + 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis3*axis3*cos_theta + 1.0*a3*axis3*axis3*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,3) = r1;
    Jrm.insert(4,4) = r0;
    Jrm.insert(4,5) = r3;
    Jrm.insert(4,6) = -r2;
    Jrm.insert(4,10) = 0.5*a0*dt*axis1*axis1*cos_theta - 1.0*a0*axis1*axis1*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*r1 + 0.5*a2*dt*axis1*axis3*cos_theta - 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis2*cos_theta + 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(4,11) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*r2 + 0.5*a2*dt*axis2*axis3*cos_theta - 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis2*axis2*cos_theta + 1.0*a3*axis2*axis2*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,12) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w - 0.5*a1*dt*r3 + 0.5*a2*dt*axis3*axis3*cos_theta - 1.0*a2*axis3*axis3*sin_theta_over_norm_w + a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(5,3) = r2;
    Jrm.insert(5,4) = -r3;
    Jrm.insert(5,5) = r0;
    Jrm.insert(5,6) = r1;
    Jrm.insert(5,10) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r1 + 0.5*a3*dt*axis1*axis1*cos_theta - 1.0*a3*axis1*axis1*sin_theta_over_norm_w + a3*norm_w*sin_theta;
    Jrm.insert(5,11) = 0.5*a0*dt*axis2*axis2*cos_theta - 1.0*a0*axis2*axis2*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*axis2*axis3*cos_theta + 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r2 + 0.5*a3*dt*axis1*axis2*cos_theta - 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(5,12) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w - 0.5*a1*dt*axis3*axis3*cos_theta + 1.0*a1*axis3*axis3*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*r3 + 0.5*a3*dt*axis1*axis3*cos_theta - 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(6,3) = r3;
    Jrm.insert(6,4) = r2;
    Jrm.insert(6,5) = -r1;
    Jrm.insert(6,6) = r0;
    Jrm.insert(6,10) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis1*axis2*cos_theta - 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis1*cos_theta + 1.0*a2*axis1*axis1*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*r1;
    Jrm.insert(6,11) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis2*axis2*cos_theta - 1.0*a1*axis2*axis2*sin_theta_over_norm_w + a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*r2;
    Jrm.insert(6,12) = 0.5*a0*dt*axis3*axis3*cos_theta - 1.0*a0*axis3*axis3*sin_theta_over_norm_w + a0*norm_w*sin_theta + 0.5*a1*dt*axis2*axis3*cos_theta - 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis3*cos_theta + 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*r3;
    // END

    // linear velocity.

    Jrm.insert(7, 7) = 1.0;
    Jrm.insert(8, 8) = 1.0;
    Jrm.insert(9, 9) = 1.0;

    // angular velocity.

    Jrm.insert(10, 10) = 1.0;
    Jrm.insert(11, 11) = 1.0;
    Jrm.insert(12, 12) = 1.0;

    for(int i=0; i<num_landmarks; i++)
    {
        Jrm.insert(13+3*i+0, 13+3*i+0) = 1.0;
        Jrm.insert(13+3*i+1, 13+3*i+1) = 1.0;
        Jrm.insert(13+3*i+2, 13+3*i+2) = 1.0;
    }

    Jrm.makeCompressed();

    J = Jrm;
}

void SLAMPrimitives::compute_h(
    const Eigen::VectorXd& X,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    double min_distance_to_camera,
    const cv::Rect& viewport,
    std::vector<int>& visible_landmarks,
    Eigen::VectorXd& h,
    Eigen::SparseMatrix<double>& J)
{
    // retrieve state dimension and number of landmarks.

    const int dim = X.size();

    if( ( dim - 13) % 3 != 0 ) throw std::runtime_error("internal error");

    const int num_landmarks = (dim - 13)/3;

    // retrieve camera position and attitude.

    const Eigen::Vector3d& camera_position = X.head<3>();

    Eigen::Quaterniond camera_attitude;
    camera_attitude.w() = X(3);
    camera_attitude.vec() = X.segment<3>(4);

    // prepare some data structures.

    visible_landmarks.clear();
    visible_landmarks.reserve(num_landmarks);

    std::vector<cv::Point3f> to_project;
    to_project.reserve(num_landmarks);

    // find which points we will try to project.

    for(int i=0; i<num_landmarks; i++)
    {
        const Eigen::Vector3d in_camera_frame = camera_attitude.inverse() * ( X.segment<3>(13 + 3*i) - camera_position );

        if( in_camera_frame.z() > 1.0e-6 && in_camera_frame.z() > min_distance_to_camera )
        {
            cv::Mat pixels = camera_matrix * ( cv::Mat_<float>(3,1) << in_camera_frame.x(), in_camera_frame.y(), in_camera_frame.z() );

            cv::Point2f pt(
                pixels.at<float>(0) / pixels.at<float>(3),
                pixels.at<float>(1) / pixels.at<float>(3) );

            if( viewport.contains(pt) )
            {
                visible_landmarks.push_back(i);

                to_project.push_back( cv::Point3f(
                    in_camera_frame.x(),
                    in_camera_frame.y(),
                    in_camera_frame.z() ));
            }
        }
    }

    const int num_visible = visible_landmarks.size();

    h.resize(2*num_visible);

    J.resize(2*num_visible, dim);
    J.setZero();
    J.reserve(20*num_visible);

    if( num_visible > 0 )
    {
        std::vector<cv::Point2f> projected;
        cv::Mat jacobian;

        cv::projectPoints(
            to_project,
            cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F),
            camera_matrix,
            distortion_coefficients,
            projected,
            jacobian);

        if( jacobian.depth() != CV_64F ) throw std::runtime_error("internal error");
        if( jacobian.channels() != 1 ) throw std::runtime_error("internal error");
        if( jacobian.rows != 2*num_visible ) throw std::runtime_error("internal error");
        if( projected.size() != num_visible) throw std::runtime_error("internal error");

        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > J1( (double*) jacobian.ptr(), 2*num_visible, 15);

        /*
        for(int i=0; i<2*num_visible; i++)
            for(int j=0; j<15; j++)
                if( std::fabs( J1(i,j) - jacobian.at<double>(i,j) ) > 1.0e-5 )
                    throw std::runtime_error("error "+std::to_string(i) + " " + std::to_string(j) );
        */

        for(int i=0; i<num_visible; i++)
        {
            // check that visible_landmarks vector is sorted (if not, filling the sparse matrix would take a long time).
            if( i > 0 && visible_landmarks[i-1] >= visible_landmarks[i] ) throw std::logic_error("internal error");

            const int j = visible_landmarks[i];

            // set jacobian coefficients.

            Eigen::Matrix<double, 3, 10> J2;
            computeJacobianOfWorld2CameraTransformation(X, j, J2 );

            Eigen::Matrix<double, 2, 10> J3 = J1.block<2, 3>(2*i, 3) * J2;

            for(int a=0; a<2; a++)
            {
                for(int b=0; b<3; b++)
                    J.insert(2*i+a, b) = J3(a, b);

                for(int b=0; b<4; b++)
                    J.insert(2*i+a, 3+b) = J3(a, 3+b);

                for(int b=0; b<3; b++)
                    J.insert(2*i+a, 13+3*j+b) = J3(a, 7+b);
            }
        }

    }

    J.makeCompressed();
}

void SLAMPrimitives::computeJacobianOfWorld2CameraTransformation(
    const Eigen::VectorXd& X,
    int landmark,
    Eigen::Matrix<double, 3, 10>& J)
{
    Eigen::Quaterniond camera_attitude;
    camera_attitude.w() = X(3);
    camera_attitude.vec() = X.segment<3>(4);

    const Eigen::Matrix3d R = camera_attitude.inverse().toRotationMatrix();

    const double x1 = X(0);
    const double x2 = X(1);
    const double x3 = X(2);

    const double a0 = X(3);
    const double a1 = X(4);
    const double a2 = X(5);
    const double a3 = X(6);

    const double y1 = X(13+3*landmark+0);
    const double y2 = X(13+3*landmark+1);
    const double y3 = X(13+3*landmark+2);

    J.block<3,3>(0, 0) = -R;

    J(0, 3) = -2*a2*(-x3 + y3) + 2*a3*(-x2 + y2);
    J(0, 4) = 2*a2*(-x2 + y2) + 2*a3*(-x3 + y3);
    J(0, 5) = -2*a0*(-x3 + y3) + 2*a1*(-x2 + y2) - 4*a2*(-x1 + y1);
    J(0, 6) = 2*a0*(-x2 + y2) + 2*a1*(-x3 + y3) - 4*a3*(-x1 + y1);

    J(1, 3) = 2*a1*(-x3 + y3) - 2*a3*(-x1 + y1);
    J(1, 4) = 2*a0*(-x3 + y3) - 4*a1*(-x2 + y2) + 2*a2*(-x1 + y1);
    J(1, 5) = 2*a1*(-x1 + y1) + 2*a3*(-x3 + y3);
    J(1, 6) = -2*a0*(-x1 + y1) + 2*a2*(-x3 + y3) - 4*a3*(-x2 + y2);

    J(2, 3) = -2*a1*(-x2 + y2) + 2*a2*(-x1 + y1);
    J(2, 4) = -2*a0*(-x2 + y2) - 4*a1*(-x3 + y3) + 2*a3*(-x1 + y1);
    J(2, 5) = 2*a0*(-x1 + y1) - 4*a2*(-x3 + y3) + 2*a3*(-x2 + y2);
    J(2, 6) = 2*a1*(-x1 + y1) + 2*a2*(-x2 + y2);

    J.block<3,3>(0, 7) = R;
}

void SLAMPrimitives::convertPose(
    const cv::Mat& rodrigues,
    const cv::Mat& t,
    Eigen::Quaterniond& attitude,
    Eigen::Vector3d& position)
{
    Eigen::Vector3d rodrigues_eigen;
    if( rodrigues.type() == CV_32F )
    {
        rodrigues_eigen.x() = rodrigues.at<float>(0);
        rodrigues_eigen.y() = rodrigues.at<float>(1);
        rodrigues_eigen.z() = rodrigues.at<float>(2);
    }
    else if( rodrigues.type() == CV_64F )
    {
        rodrigues_eigen.x() = rodrigues.at<double>(0);
        rodrigues_eigen.y() = rodrigues.at<double>(1);
        rodrigues_eigen.z() = rodrigues.at<double>(2);
    }

    Eigen::Vector3d t_eigen;
    if( t.type() == CV_32F )
    {
        t_eigen.x() = t.at<float>(0);
        t_eigen.y() = t.at<float>(1);
        t_eigen.z() = t.at<float>(2);
    }
    else if( t.type() == CV_64F )
    {
        t_eigen.x() = t.at<double>(0);
        t_eigen.y() = t.at<double>(1);
        t_eigen.z() = t.at<double>(2);
    }

    const double norm = rodrigues_eigen.norm();

    if( norm > 1.0e-8 )
    {
        attitude.vec() = -sin(0.5*norm) * rodrigues_eigen / norm;
        attitude.w() = cos(0.5*norm);
    }
    else
    {
        attitude.setIdentity();
    }

    position = -( attitude * t_eigen );
}
