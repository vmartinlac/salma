#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>

class Image
{
public:

    Image()
    {
        mValid = false;
    }

    void setInvalid()
    {
        mValid = false;
    }

    void setValid(double timestamp, const cv::Mat& frame)
    {
        mValid = true;
        mTimestamp = timestamp;
        mNumberOfFrames = 1;
        mFrames.at(0) = frame;
    }

    void setValid(double timestamp, const cv::Mat& left_frame, const cv::Mat& right_frame)
    {
        mValid = true;
        mTimestamp = timestamp;
        mNumberOfFrames = 2;
        mFrames.at(0) = left_frame;
        mFrames.at(1) = right_frame;
    }

    int getNumberOfFrames()
    {
        if( mValid )
        {
            return mNumberOfFrames;
        }
        else
        {
            return 0;
        }
    }

    double getTimestamp()
    {
        if( mValid )
        {
            return mTimestamp;
        }
        else
        {
            return 0.0;
        }
    }

    cv::Mat& frame(int idx=0)
    {
        if( mValid == false || idx < 0 || idx >= mNumberOfFrames )
        {
            throw std::runtime_error("Invalid image or incorrect frame number");
        }
        else
        {
            return mFrames.at(idx);
        }
    }

protected:

    bool mValid;
    double mTimestamp;
    int mNumberOfFrames;
    std::array<cv::Mat, 2> mFrames;
};

class VideoSource
{

public:

    VideoSource();
    virtual ~VideoSource();

    virtual std::string getHumanName() = 0;

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void trigger() = 0;
    virtual void read(Image& image) = 0;

    virtual int getNumberOfCameras() = 0;
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;

class VideoSystem
{
public:

    bool initialize();
    void finalize();
    static VideoSystem& instance();

public:

    VideoSystem();
    ~VideoSystem();

    VideoSourcePtr createMonoAvtVideoSource(int camera_idx);
    VideoSourcePtr createStereoAvtVideoSource(int left_camera_idx, int right_camera_id);
    VideoSourcePtr createOpenCVVideoSource(const std::string& filename);
    VideoSourcePtr createOpenCVVideoSource(int id);
    VideoSourcePtr createVideoSourceFromMonoRecording(const std::string& path);
    VideoSourcePtr createVideoSourceFromStereoRecording(const std::string& path);

    int getNumberOfAvtCameras();
    std::string getNameOfAvtCamera(int idx);

    bool lockAvtRig()
    {
        return mAvtMutex.try_lock();
    }

    void unlockAvtRig()
    {
        mAvtMutex.unlock();
    }

    void triggerAvtRig();

private:

    std::mutex mAvtMutex;

    static VideoSystem* mInstance;
};

