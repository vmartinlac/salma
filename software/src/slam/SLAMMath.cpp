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

