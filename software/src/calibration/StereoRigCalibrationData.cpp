#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "StereoRigCalibrationData.h"
#include "Serialization.h"

StereoRigCalibrationData::StereoRigCalibrationData()
{
    id = -1;
}

/*
bool StereoRigCalibrationData::saveToFile(const std::string& path)
{
    bool ok = true;

    QJsonDocument doc;

    try
    {
        QJsonObject obj;
        obj["left_camera_to_rig"] = Serialization::serializePose(left_camera_to_rig);
        obj["right_camera_to_rig"] = Serialization::serializePose(right_camera_to_rig);
        obj["name"] = name.c_str();

        doc.setObject(obj);
    }
    catch(SerializationError& err)
    {
        ok = false;
    }

    QFile file;

    if(ok)
    {
        file.setFileName(path.c_str());
        ok = file.open(QIODevice::WriteOnly);
    }

    if(ok)
    {
        file.write(doc.toJson());
        file.close();
    }

    return ok;
}

bool StereoRigCalibrationData::loadFromFile(const std::string& path)
{
    bool ok = true;
    QFile file;
    QByteArray buff;
    QJsonDocument doc;
    QJsonObject root;

    if(ok)
    {
        file.setFileName(path.c_str());
        ok = file.open(QIODevice::ReadOnly);
    }

    if(ok)
    {
        buff = file.readAll();
        file.close();
    }

    if(ok)
    {
        doc = QJsonDocument::fromJson(buff);
        ok = (doc.isNull() == false) && doc.isObject();
    }

    if(ok)
    {
        root = doc.object();
        try
        {
            left_camera_to_rig = Serialization::deserializePose(root["left_camera_to_rig"]);
            right_camera_to_rig = Serialization::deserializePose(root["right_camera_to_rig"]);
            name = root["name"].toString().toStdString();
        }
        catch(SerializationError& err)
        {
            ok = false;
        }
    }

    return ok;
}
*/

Eigen::Matrix3d StereoRigCalibrationData::vectorialProductMatrix(const Eigen::Vector3d& v)
{
    /*
    b*z - c*y
    c*x - a*z
    a*y - b*x
    */

    Eigen::Matrix3d ret;
    ret <<
        0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
        
    return ret;
}

Eigen::Matrix3d StereoRigCalibrationData::computeFundamentalMatrix(int from, int to)
{
    /*
    Return F such that transpose(left_image_point) * F * right_image_point is zero.
    */

    if(from == to || from >= 2 || from < 0 || to >= 2 || to < 0) throw std::runtime_error("internal error");

    const Eigen::Matrix3d E = computeEssentialMatrix( from, to );

    const Eigen::Matrix3d inverse_left_K = cameras[to].calibration->inverseOfCalibrationMatrix();
    const Eigen::Matrix3d inverse_right_K = cameras[from].calibration->inverseOfCalibrationMatrix();

    const Eigen::Matrix3d F = inverse_left_K.transpose() * E * inverse_right_K;

    /*
    {
        Eigen::Matrix3d KL;
        Eigen::Matrix3d KR;
        cv::cv2eigen( left_camera->calibration_matrix, KL );
        cv::cv2eigen( right_camera->calibration_matrix, KR );

        const Sophus::SE3d rig_to_left = stereo_rig->left_camera_to_rig.inverse();
        const Sophus::SE3d rig_to_right = stereo_rig->right_camera_to_rig.inverse();

        for(double l=100.0; l<800.0; l+=10.0)
        {
            const Eigen::Vector3d P = stereo_rig->left_camera_to_rig.translation() + l * stereo_rig->left_camera_to_rig.rotationMatrix().col(2);

            const Eigen::Vector3d XL = KL * (rig_to_left * P);
            const Eigen::Vector3d XR = KR * (rig_to_right * P);

            std::cout << XL.transpose() * F * XR << std::endl;
        }
        exit(0);
    }
    */

    return F;
}

Eigen::Matrix3d StereoRigCalibrationData::computeEssentialMatrix(int from, int to)
{
    /*
    Return E such that transpose(left_image_point) * E * right_image_point is zero.
    */

    if(from == to || from >= 2 || from < 0 || to >= 2 || to < 0) throw std::runtime_error("internal error");

    const Sophus::SE3d transform = cameras[to].camera_to_rig.inverse() * cameras[from].camera_to_rig;

    const Eigen::Matrix3d R = transform.rotationMatrix();
    const Eigen::Vector3d t = transform.translation();

    const Eigen::Matrix3d E = vectorialProductMatrix( t ) * R;

    return E;
}

