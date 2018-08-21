#pragma once

#include <QString>
#include <QMutex>
#include <QObject>
#include <Eigen/Eigen>
#include <vector>
#include <opencv2/core.hpp>

struct SLAMOutputLandmark
{
    Eigen::Vector3d position;
};

class SLAMOutput : public QObject
{
    Q_OBJECT

public:

    SLAMOutput(QObject* parent = nullptr);

    void beginRead();
    void endRead();

    void beginWrite();
    void endWrite();

    void clear();

public:

    // string describing what the slam engine is doing.
    QString mode;
    int frame_id;

    // output debug image (basically the image from the camera annotated by the engine).
    cv::Mat image;

    // camera state.
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    // landmarks.
    std::vector<SLAMOutputLandmark> landmarks;

    // candidate landmarks.
    // TODO

signals:

    void updated();

protected:

    QMutex m_mutex;
};

inline void SLAMOutput::beginRead() { m_mutex.lock(); }
inline void SLAMOutput::endRead() { m_mutex.unlock(); }

inline void SLAMOutput::beginWrite() { m_mutex.lock(); }
inline void SLAMOutput::endWrite() { m_mutex.unlock(); }
