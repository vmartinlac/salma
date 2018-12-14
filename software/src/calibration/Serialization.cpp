#include <QJsonArray>
#include <Eigen/Eigen>
#include "Serialization.h"

SerializationError::SerializationError() : std::runtime_error("serialization error")
{
}

QJsonValue Serialization::serializePose(const Sophus::SE3d& from)
{
    QJsonArray ret;

    const double* data = from.data();

    for(int i=0; i<7; i++)
    {
        ret.push_back(data[i]);
    }

    return ret;
}

Sophus::SE3d Serialization::deserializePose(const QJsonValue& from)
{
    bool ok = true;

    QJsonArray arr;
    Sophus::SE3d ret;

    if(ok)
    {
        ok = from.isArray();
    }

    if(ok)
    {
        arr = from.toArray();
        ok = (arr.size() == 7);
    }

    if(ok)
    {
        for(int i=0; ok && i<7; i++)
        {
            ok = arr[i].isDouble();

            if(ok)
            {
                ret.data()[i] = arr[i].toDouble();
            }
        }

        ret.normalize();
    }

    if(ok == false)
    {
        throw SerializationError();
    }

    return ret;
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
    bool ok = true;

    QJsonArray arr;
    cv::Mat ret(3, 3, CV_64F);

    if(ok)
    {
        ok = from.isArray();
    }

    if(ok)
    {
        arr = from.toArray();
        ok = (arr.size() == 9);
    }

    if(ok)
    {
        for(int i=0; ok && i<9; i++)
        {
            ok = arr[i].isDouble();

            if(ok)
            {
                ret.at<double>(i/3, i%3) = arr[i].toDouble();
            }
        }
    }

    if(ok == false)
    {
        throw SerializationError();
    }

    return ret;
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
    bool ok = true;

    QJsonArray arr;
    cv::Mat ret;

    if(ok)
    {
        ok = from.isArray();
    }

    if(ok)
    {
        arr = from.toArray();
        ret = cv::Mat(1, arr.size(), CV_64F);

        for(int i=0; ok && i<arr.size(); i++)
        {
            ok = arr[i].isDouble();

            if(ok)
            {
                ret.at<double>(0, i) = arr[i].toDouble();
            }
        }
    }

    if(ok == false)
    {
        throw SerializationError();
    }

    return ret;
}


QJsonValue Serialization::serializeSize(const cv::Size& size)
{
    QJsonArray ret;
    ret.push_back( size.width );
    ret.push_back( size.height );
    return ret;
}

cv::Size Serialization::deserializeSize(const QJsonValue& from)
{
    bool ok = true;

    QJsonArray arr;
    cv::Size ret;

    if(ok)
    {
        ok = from.isArray();
    }

    if(ok)
    {
        arr = from.toArray();
        ok = (arr.size() == 2);
    }

    if(ok)
    {
        ok = arr[0].isDouble() && arr[1].isDouble();
    }

    if(ok)
    {
        ret.width = arr[0].toDouble();
        ret.height = arr[1].toDouble();
    }

    if(ok == false)
    {
        throw SerializationError();
    }

    return ret;
}

