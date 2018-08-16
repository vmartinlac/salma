#pragma once

#include <QString>
#include <QMutex>
#include <QObject>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>

class SLAMOutput : public QObject
{
    Q_OBJECT

public:

    SLAMOutput(QObject* parent = nullptr);

    void beginRead();
    void endRead();

    void beginWrite();
    void endWrite();

public:

    QString mode;
    cv::Mat image;

    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

signals:

    void updated();

protected:

    QMutex m_mutex;
};

inline void SLAMOutput::beginRead() { m_mutex.lock(); }
inline void SLAMOutput::endRead() { m_mutex.unlock(); }

inline void SLAMOutput::beginWrite() { m_mutex.lock(); }
inline void SLAMOutput::endWrite() { m_mutex.unlock(); }
