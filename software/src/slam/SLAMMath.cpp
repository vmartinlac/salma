#include "SLAMMath.h"

Eigen::Vector4d SLAMMath::convert(const Eigen::Quaterniond& from)
{
    Eigen::Vector4d ret;

    ret(0) = from.x();
    ret(1) = from.y();
    ret(2) = from.z();
    ret(3) = from.w();

    return ret;
}

Eigen::Quaterniond SLAMMath::convert(const Eigen::Vector4d& from)
{
    Eigen::Quaterniond ret;

    ret.vec() = from.head<3>();
    ret.w() = from(3);

    return ret;
}

void SLAMMath::computeQuaternionVectorProduct(
    const Eigen::Vector4d& q,
    const Eigen::Vector3d& v,
    Eigen::Vector3d* result,
    Eigen::Matrix<double, 3, 7>* J)
{
    const double qi = q(0);
    const double qj = q(1);
    const double qk = q(2);
    const double qr = q(3);

    const double vi = v(0);
    const double vj = v(1);
    const double vk = v(2);

    if(result)
    {
        auto& ret = *result;

        ret(0) = vi*(-2*(pow(qj, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1) + 2*vj*(qi*qj - qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*vk*(qi*qk + qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        ret(1) = 2*vi*(qi*qj + qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vj*(-2*(pow(qi, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1) + 2*vk*(-qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        ret(2) = 2*vi*(qi*qk - qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*vj*(qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vk*(-2*(pow(qi, 2) + pow(qj, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1);
    }

    if(J)
    {
        auto& Jref = *J;

        Jref(0, 0) = -2*qi*vi*(-2*pow(qj, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi*vj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi*vk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*qk*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(0, 1) = 2*qi*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*vj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj*vk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vi*(4*qj*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(0, 2) = 2*qi*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*vj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk*vk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vi*(4*qk*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(0, 3) = 2*qj*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 2*qk*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 2*qr*vi*(-2*pow(qj, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qr*vj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qr*vk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
        Jref(0, 4) = -2*(pow(qj, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
        Jref(0, 5) = 2*(qi*qj - qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(0, 6) = 2*(qi*qk + qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(1, 0) = -4*qi*vi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi*vk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 2*qr*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vj*(4*qi*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(1, 1) = 2*qi*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*vi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qj*vj*(-2*pow(qi, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj*vk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(1, 2) = 2*qj*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*vi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk*vk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vj*(4*qk*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(1, 3) = -2*qi*vk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*qk*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*vi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr*vj*(-2*pow(qi, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qr*vk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
        Jref(1, 4) = 2*(qi*qj + qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(1, 5) = -2*(pow(qi, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
        Jref(1, 6) = 2*(-qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(2, 0) = -4*qi*vi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi*vj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*qr*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vk*(4*qi*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(2, 1) = -4*qj*vi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj*vj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 2*qr*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + vk*(4*qj*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)));
        Jref(2, 2) = 2*qi*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 2*qj*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*vi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk*vj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qk*vk*(-2*pow(qi, 2) - 2*pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
        Jref(2, 3) = 2*qi*vj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 2*qj*vi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*vi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qr*vj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr*vk*(-2*pow(qi, 2) - 2*pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
        Jref(2, 4) = 2*(qi*qk - qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(2, 5) = 2*(qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
        Jref(2, 6) = -2*(pow(qi, 2) + pow(qj, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
    }
}

void SLAMMath::computeQuaternionQuaternionProduct(
    const Eigen::Vector4d& P,
    const Eigen::Vector4d& Q,
    Eigen::Vector4d* result,
    Eigen::Matrix<double, 4, 8>* J)
{
    const double pi = P(0);
    const double pj = P(1);
    const double pk = P(2);
    const double pr = P(3);

    const double qi = Q(0);
    const double qj = Q(1);
    const double qk = Q(2);
    const double qr = Q(3);

    if(result)
    {
        auto& ret = *result;

        ret(0) = pr * qi + qr * pi + (pj*qk - pk*qj);
        ret(1) = pr * qj + qr * pj + (pk*qi - pi*qk);
        ret(2) = pr * qk + qr * pk + (pi*qj - pj*qi);
        ret(3) = pr*qr - pi*qi - pj*qj - pk*qk;
    }

    if(J)
    {
        auto& Jref = *J; 

        Jref( 0, 0 ) = qk;
        Jref( 0, 1 ) = qj;
        Jref( 0, 2 ) = -qi;
        Jref( 0, 3 ) = qr;
        Jref( 0, 4 ) = pk;
        Jref( 0, 5 ) = -pj;
        Jref( 0, 6 ) = pi;
        Jref( 0, 7 ) = pr;
        Jref( 1, 0 ) = -qj;
        Jref( 1, 1 ) = qk;
        Jref( 1, 2 ) = qr;
        Jref( 1, 3 ) = qi;
        Jref( 1, 4 ) = pj;
        Jref( 1, 5 ) = pk;
        Jref( 1, 6 ) = -pr;
        Jref( 1, 7 ) = pi;
        Jref( 2, 0 ) = qi;
        Jref( 2, 1 ) = -qr;
        Jref( 2, 2 ) = qk;
        Jref( 2, 3 ) = qj;
        Jref( 2, 4 ) = -pi;
        Jref( 2, 5 ) = pr;
        Jref( 2, 6 ) = pk;
        Jref( 2, 7 ) = pj;
        Jref( 3, 0 ) = -qr;
        Jref( 3, 1 ) = -qi;
        Jref( 3, 2 ) = -qj;
        Jref( 3, 3 ) = qk;
        Jref( 3, 4 ) = -pr;
        Jref( 3, 5 ) = -pi;
        Jref( 3, 6 ) = -pj;
        Jref( 3, 7 ) = pk;
    }
}

void SLAMMath::rigidTransform(
    const Sophus::SE3d& mu_transform,
    const Eigen::Matrix<double, 7, 7>& sigma_transform,
    const Eigen::Vector3d& mu_point,
    const Eigen::Matrix3d& sigma_point,
    Eigen::Vector3d& mu_transformed,
    Eigen::Matrix<double, 3, 10>& sigma_transformed)
{
    Eigen::Matrix<double, 10, 10> J;
    rigidTransform(
        mu_transform,
        mu_point,
        mu_transformed,
        J);

    Eigen::Matrix<double, 10, 10> sigma_input;
    sigma_input.setZero();
    sigma_input.block<3,3>(0,0) = sigma_point;
    sigma_input.block<7,7>(3,3) = sigma_transform;

    const Eigen::Matrix<double, 10, 10> sigma_output = J * sigma_input * J.transpose();
    //std::cout << sigma_input << std::endl;
    //std::cout << sigma_output << std::endl;

    //std::cout << "AAA " << (mu_transformed - mu_transform * mu_point).norm() << std::endl;
    sigma_transformed = sigma_output.topRows<3>();
}

void SLAMMath::rigidTransform(
    const Sophus::SE3d& transform,
    const Eigen::Vector3d& point,
    Eigen::Vector3d& result,
    Eigen::Matrix<double, 10, 10>& J)
{
    const double v_x = point.x();
    const double v_y = point.y();
    const double v_z = point.z();

    const double t_x = transform.translation().x();
    const double t_y = transform.translation().y();
    const double t_z = transform.translation().z();

    const double q_x = transform.unit_quaternion().vec().x();
    const double q_y = transform.unit_quaternion().vec().y();
    const double q_z = transform.unit_quaternion().vec().z();
    const double q_w = transform.unit_quaternion().w();

    result(0) = t_x + v_x*(-2*(pow(q_y, 2) + pow(q_z, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1) + 2*v_y*(-q_w*q_z + q_x*q_y)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*v_z*(q_w*q_y + q_x*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    result(1) = t_y + 2*v_x*(q_w*q_z + q_x*q_y)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + v_y*(-2*(pow(q_x, 2) + pow(q_z, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1) + 2*v_z*(-q_w*q_x + q_y*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    result(2) = t_z + 2*v_x*(-q_w*q_y + q_x*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*v_y*(q_w*q_x + q_y*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + v_z*(-2*(pow(q_x, 2) + pow(q_y, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1);

    /*
    result(3) = t_x;
    result(4) = t_y;
    result(5) = t_z;
    result(6) = q_x;
    result(7) = q_y;
    result(8) = q_z;
    result(9) = q_w;
    */

    J(0, 0) = -2*(pow(q_y, 2) + pow(q_z, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1;
    J(0, 1) = 2*(-q_w*q_z + q_x*q_y)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(0, 2) = 2*(q_w*q_y + q_x*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(0, 3) = 1;
    J(0, 4) = 0;
    J(0, 5) = 0;
    J(0, 6) = -2*q_x*v_x*(-2*pow(q_y, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x*v_y*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x*v_z*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_z*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(0, 7) = 2*q_w*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_x*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*v_y*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y*v_z*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + v_x*(4*q_y*(pow(q_y, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(0, 8) = -2*q_w*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_x*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*v_y*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z*v_z*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + v_x*(4*q_z*(pow(q_y, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(0, 9) = -2*q_w*v_x*(-2*pow(q_y, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_w*v_y*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_w*v_z*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 2*q_z*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(1, 0) = 2*(q_w*q_z + q_x*q_y)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(1, 1) = -2*(pow(q_x, 2) + pow(q_z, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1;
    J(1, 2) = 2*(-q_w*q_x + q_y*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(1, 3) = 0;
    J(1, 4) = 1;
    J(1, 5) = 0;
    J(1, 6) = -2*q_w*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_x*v_x*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x*v_z*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + v_y*(4*q_x*(pow(q_x, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(1, 7) = 2*q_x*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*v_x*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_y*v_y*(-2*pow(q_x, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y*v_z*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(1, 8) = 2*q_w*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_y*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*v_x*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z*v_z*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + v_y*(4*q_z*(pow(q_x, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(1, 9) = -4*q_w*v_x*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_w*v_y*(-2*pow(q_x, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_w*v_z*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_x*v_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_z*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(2, 0) = 2*(-q_w*q_y + q_x*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(2, 1) = 2*(q_w*q_x + q_y*q_z)/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(2, 2) = -2*(pow(q_x, 2) + pow(q_y, 2))/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 1;
    J(2, 3) = 0;
    J(2, 4) = 0;
    J(2, 5) = 1;
    J(2, 6) = 2*q_w*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_x*v_x*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x*v_y*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + v_z*(4*q_x*(pow(q_x, 2) + pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(2, 7) = -2*q_w*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*v_x*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y*v_y*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + v_z*(4*q_y*(pow(q_x, 2) + pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)));
    J(2, 8) = 2*q_x*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) + 2*q_y*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*v_x*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z*v_y*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_z*v_z*(-2*pow(q_x, 2) - 2*pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(2, 9) = -4*q_w*v_x*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_w*v_y*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_w*v_z*(-2*pow(q_x, 2) - 2*pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_x*v_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 2*q_y*v_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(3, 0) = 0;
    J(3, 1) = 0;
    J(3, 2) = 0;
    J(3, 3) = 1;
    J(3, 4) = 0;
    J(3, 5) = 0;
    J(3, 6) = 0;
    J(3, 7) = 0;
    J(3, 8) = 0;
    J(3, 9) = 0;
    J(4, 0) = 0;
    J(4, 1) = 0;
    J(4, 2) = 0;
    J(4, 3) = 0;
    J(4, 4) = 1;
    J(4, 5) = 0;
    J(4, 6) = 0;
    J(4, 7) = 0;
    J(4, 8) = 0;
    J(4, 9) = 0;
    J(5, 0) = 0;
    J(5, 1) = 0;
    J(5, 2) = 0;
    J(5, 3) = 0;
    J(5, 4) = 0;
    J(5, 5) = 1;
    J(5, 6) = 0;
    J(5, 7) = 0;
    J(5, 8) = 0;
    J(5, 9) = 0;
    J(6, 0) = 0;
    J(6, 1) = 0;
    J(6, 2) = 0;
    J(6, 3) = 0;
    J(6, 4) = 0;
    J(6, 5) = 0;
    J(6, 6) = 1;
    J(6, 7) = 0;
    J(6, 8) = 0;
    J(6, 9) = 0;
    J(7, 0) = 0;
    J(7, 1) = 0;
    J(7, 2) = 0;
    J(7, 3) = 0;
    J(7, 4) = 0;
    J(7, 5) = 0;
    J(7, 6) = 0;
    J(7, 7) = 1;
    J(7, 8) = 0;
    J(7, 9) = 0;
    J(8, 0) = 0;
    J(8, 1) = 0;
    J(8, 2) = 0;
    J(8, 3) = 0;
    J(8, 4) = 0;
    J(8, 5) = 0;
    J(8, 6) = 0;
    J(8, 7) = 0;
    J(8, 8) = 1;
    J(8, 9) = 0;
    J(9, 0) = 0;
    J(9, 1) = 0;
    J(9, 2) = 0;
    J(9, 3) = 0;
    J(9, 4) = 0;
    J(9, 5) = 0;
    J(9, 6) = 0;
    J(9, 7) = 0;
    J(9, 8) = 0;
    J(9, 9) = 1;
}

void SLAMMath::computeJacobianOfQuaternionToRotationMatrix(
    const Eigen::Quaterniond& q,
    Eigen::Matrix<double, 9, 4>& J)
{
    const double q_x = q.vec().x();
    const double q_y = q.vec().y();
    const double q_z = q.vec().z();
    const double q_w = q.w();

    J(0, 0) = -2*q_x*(-2*pow(q_y, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(0, 1) = 4*q_y*(pow(q_y, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(0, 2) = 4*q_z*(pow(q_y, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(0, 3) = -2*q_w*(-2*pow(q_y, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(1, 0) = -4*q_x*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(1, 1) = 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(1, 2) = -2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(1, 3) = -4*q_w*(-q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(2, 0) = -4*q_x*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(2, 1) = 2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(2, 2) = 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(2, 3) = -4*q_w*(q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(3, 0) = -4*q_x*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(3, 1) = 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(3, 2) = 2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(3, 3) = -4*q_w*(q_w*q_z + q_x*q_y)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(4, 0) = 4*q_x*(pow(q_x, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(4, 1) = -2*q_y*(-2*pow(q_x, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(4, 2) = 4*q_z*(pow(q_x, 2) + pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(4, 3) = -2*q_w*(-2*pow(q_x, 2) - 2*pow(q_z, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(5, 0) = -2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_x*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(5, 1) = -4*q_y*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(5, 2) = 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(5, 3) = -4*q_w*(-q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(6, 0) = -4*q_x*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(6, 1) = -2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_y*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(6, 2) = 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(6, 3) = -4*q_w*(-q_w*q_y + q_x*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(7, 0) = 2*q_w/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_x*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(7, 1) = -4*q_y*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_z/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(7, 2) = 2*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2)) - 4*q_z*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(7, 3) = -4*q_w*(q_w*q_x + q_y*q_z)/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) + 2*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(8, 0) = 4*q_x*(pow(q_x, 2) + pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_x/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(8, 1) = 4*q_y*(pow(q_x, 2) + pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2) - 4*q_y/(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2));
    J(8, 2) = -2*q_z*(-2*pow(q_x, 2) - 2*pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
    J(8, 3) = -2*q_w*(-2*pow(q_x, 2) - 2*pow(q_y, 2))/pow(pow(q_w, 2) + pow(q_x, 2) + pow(q_y, 2) + pow(q_z, 2), 2);
}

