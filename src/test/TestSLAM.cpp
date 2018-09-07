#include "SLAMPrimitives.h"
#include "TestSLAM.h"

void TestSLAM::test_f()
{
    // check that compute_f predict the good future state.

    Eigen::VectorXd X(13+3*2);
    X.head<13>() <<
        1.0, 1.0, 1.0,
        0.0, 1.0, 1.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, M_PI*0.5;

    X.segment<4>(3).normalize();

    X.segment<3>(13) << 0.0, 0.0, -5;
    X.segment<3>(13+3) << 1.0, 0.0, -5;

    Eigen::VectorXd f;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_f(X, 1.0, f, J);


    QVERIFY( J.rows() == X.size() && J.cols() == X.size() );
    QVERIFY( f.size() == X.size() );

    QVERIFY( ( X.tail(6) - f.tail(6) ).norm() < 1.0e-7 );
    QVERIFY( ( X.segment<6>(7) - f.segment<6>(7) ).norm() < 1.0e-7 );

    Eigen::MatrixXd FDJ;
    finite_differences_f(X, 1.0, FDJ);
    FDJ -= J;
    std::cout << FDJ << std::endl;
}

void TestSLAM::test_h()
{
    Eigen::VectorXd X(13+3*3);
    X <<
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,

        //1.0, 0.0, 0.0,
        0.0, 0.0, 0.0,

        //0.0, 0.0, M_PI*0.5,
        0.0, 0.0, 0.0,

        0.0, 0.0, -5.0,
        1.0, 0.0, -5.0,
        0.0, 0.0, 5.0;

    //X.segment<4>(3).normalize();

    cv::Mat K;
    const double alpha = M_PI*15.0/180.0;
    K = (cv::Mat_<float>(3,3,CV_64F) << 320.0/tan(alpha), 0.0, 320, 0.0, 240.0/tan(alpha), 240, 0.0, 0.0, 1.0);
    std::vector<int> lm;
    Eigen::VectorXd h;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_h(
        X,
        K,
        cv::Mat(),
        1.0e-2,
        cv::Rect(0,0,640,480),
        lm,
        h,
        J);

    QVERIFY( lm.size() == 2 );
    QVERIFY( lm[0] == 0 && lm[1] == 1 );

    Eigen::MatrixXd J2;
    finite_differences_h(
        X,
        K,
        cv::Mat(),
        1.0e-2,
        cv::Rect(0,0,640,480),
        lm,
        J2);
    std::cout << J - J2 << std::endl;
}

void TestSLAM::finite_differences_f(const Eigen::VectorXd& X, double dt, Eigen::MatrixXd& J)
{
    const double eps = 1.0e-6;

    Eigen::SparseMatrix<double> tmp;

    Eigen::VectorXd fref;
    SLAMPrimitives::compute_f(X, dt, fref, tmp);

    J.resize(X.size(), X.size());
    for(int i=0; i<X.size(); i++)
    {
        Eigen::VectorXd XX = X;
        XX(i) += eps;

        Eigen::VectorXd f;
        SLAMPrimitives::compute_f(XX, dt, f, tmp);

        J.col(i) = (f - fref) * (1.0/eps);
    }
}

void TestSLAM::finite_differences_h(
    const Eigen::VectorXd& X,
    const cv::Mat& K,
    const cv::Mat& dist,
    double min_dist,
    const cv::Rect& viewport,
    const std::vector<int>& lms,
    Eigen::MatrixXd& J)
{
    Eigen::SparseMatrix<double> tmp;
    Eigen::VectorXd h0;
    std::vector<int> lms2;
    SLAMPrimitives::compute_h( X, K, dist, min_dist, viewport, lms2, h0, tmp );

    QVERIFY( lms2.size() == lms.size() );
    for(int i=0; i<lms2.size(); i++)
        QVERIFY( lms2[i] == lms[i] );

    J.resize(h0.size(), X.size());

    const double eps = 1.0e-3;

    for(int i=0; i<X.size(); i++)
    {
        Eigen::VectorXd XX = X;
        XX(i) += eps;

        std::vector<int> lms3;
        Eigen::VectorXd h1;
        SLAMPrimitives::compute_h( XX, K, dist, min_dist, viewport, lms3, h1, tmp );

        QVERIFY( lms3.size() == lms.size() );
        for(int i=0; i<lms3.size(); i++)
            QVERIFY( lms3[i] == lms[i] );

        J.col(i) = (h1 - h0) * (1.0/eps);
    }
}

QTEST_MAIN(TestSLAM)

