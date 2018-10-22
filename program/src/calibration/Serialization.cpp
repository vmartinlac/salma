#include <QJsonArray>
#include <Eigen/Eigen>
#include "Serialization.h"

SerializationError::SerializationError() : std::runtime_error("serialization error")
{
}

QJsonValue Serialization::serializePose(const Sophus::SE3d& from)
{
    QJsonArray ret;

    Eigen::Matrix<double, 7, 1> m = from.params();

    for(int i=0; i<7; i++)
    {
        ret.push_back(m(i));
    }

    return ret;
}

Sophus::SE3d Serialization::deserializePose(const QJsonValue& from)
{
    throw SerializationError();
}


QJsonValue Serialization::serializeCalibrationMatrix(const cv::Mat& from)
{
    QJsonArray ret;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            ret.push_back( from.at<double>(i,j) );
        }
    }

    return ret;
}

cv::Mat Serialization::deserializeCalibrationMatrix(const QJsonValue& from)
{
    throw SerializationError();
}


QJsonValue Serialization::serializeDistortionCoefficients(const cv::Mat& from)
{
    QJsonArray ret;

    for(int i=0; i<from.rows; i++)
    {
        for(int j=0; j<from.cols; j++)
        {
            ret.push_back( from.at<double>(i,j) );
        }
    }

    return ret;
}

cv::Mat Serialization::deserializeDistortionCoefficients(const QJsonValue& from)
{
    throw SerializationError();
}


QJsonValue Serialization::serializeSize(const cv::Size& size)
{
    QJsonArray ret;
    ret.push_back( size.width );
    ret.push_back( size.height );
    return ret;
}

cv::Size Serialization::deserializeSize(const QJsonValue& size)
{
    throw SerializationError();
}

