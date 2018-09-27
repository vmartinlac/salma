#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>

/*
Naming conventions:
keyframe or keyFrame or KeyFrame or KEYFRAME.
keypoint or keyPoint or KeyPoint or KEYPOINT.
mappoint or mapPoint or MapPoint or MAPPOINT.
*/

class Pose
{
public:
  Eigen::Vector3d position;
  Eigen::Quaterniond attitude;

  void setIdentity()
  {
    frame->pose.attitude.setIdentity();
    frame->pose.position.setZero();
  }

  Pose invert()
  {
    Pose ret;

    ret.position = -attitude.invert() * position;
    ret.attitude = attitude.invert();

    return ret;
  }

  Pose operator*(const Pose& o)
  {
    Pose ret;

    ret.attitude = attitude * o.attitude;
    ret.position = attitude * o.position + position;

    return ret;
  }
};

class MapPoint
{
public:
  int id;
  Eigen::Vector3d position;
  cv::Mat last_descriptor;
  int track_count;
};

typedef std::shared_ptr<MapPoint> MapPointPtr;

class KeyPoint
{
public:
  cv::KeyPoint keypoint;
  cv::Point2f undistorted;
  cv::Point2f normalized;
  cv::Mat descriptor;
  MapPointPtr mappoint;
};

class View
{
public:
  cv::Mat image;
  double timestamp;
  std::vector<KeyPoint> keypoints;
};

class KeyFrame;

typedef std::shared_ptr<KeyFrame> KeyFramePtr;

class KeyFrame
{
public:
  int id;
  double timestamp;
  View left_view;
  View right_view;
  Pose keyframe_to_first_keyframe;
  KeyFramePtr previous_keyframe;
};

class Engine
{
public:

  void processFrame()
  {
    // capture des images
    // détection et description des points d'intérêt 
    // prédiction nouvelle pose
    // matching mappoint to keypoint par projection
    // calcul de la pose par PnP
    // matching keypoint to keypoint par contrainte épipolaires et triangulation
    // bundle ajustment
  }

protected:

  enum Mode
  {
    MODE_LOST,
    MODE_SLAM
  };

protected:

  KeyFramePtr mCurrentKeyFrame;
  KeyFramePtr mLastKeyFrame;
  int mNextKeyFrameId;
  int mNextMapPointId;

  Pose mLeftViewToRightView;
  Pose mLeftViewToKeyFrame;
  Pose mRightViewToKeyFrame;
  Mode mMode;
};

class PosePredictor
{
public:

  enum MotionModel
  {
    CONSTANT_VELOCITY,
    CONSTANT_ACCELERATION,
    NO_MOTION
  };

  bool run(KeyFramePtr frame, MotionModel motion_model)
  {
    switch(motion_model)
    {
    case NO_MOTION:
      return runNoMotion(frame);
    case CONSTANT_VELOCITY:
      return runConstantVelocity(frame);
    case CONSTANT_ACCELERATION:
      return runConstantAcceleration(frame);
    default:
      throw std::runtime_error("internal error");
    }
  }

protected:

  bool runNoMotion(KeyFramePtr frame)
  {
    KeyFramePtr f1 = frame->previous_keyframe;

    if( f1 )
    {
      frame->pose = f1->pose;
    }
    else
    {
      frame->pose.setIdentity();
    }
  }

  bool runConstantVelocity(KeyFramePtr frame)
  {
    KeyFramePtr f1 = frame->previous_keyframe;

    if( f1 )
    {
      KeyFramePtr other = f1->previous_keyframe;

      while( other && f1->timestamp - other->timestamp < frame->timestamp - f1->timestamp )
      {
        other = other->previous_keyframe;
      }

      if(other)
      {
        Pose& from = other->pose;
        Pose& to = f1->pose;

        const Pose delta = to * from.invert();

        const double t0 = (frame->timestamp - f1->timestamp) / (f1->timestamp - other->timestamp);
        const double t = std::max(0.0, std::min(1.0, t));

        Pose new_delta = delta.slerp(t);
      }
      else
      {
        frame->pose = f1->pose;
      }
    }
    else
    {
      frame->pose.setIdentity();
    }
  }

  bool runConstantAcceleration(KeyFramePtr frame)
  {
    throw std::runtime_error("not implemented");
  }

};

