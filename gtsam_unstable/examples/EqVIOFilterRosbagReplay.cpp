/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOFilterRosbagReplay.cpp
/// @brief Optional ROS1 rosbag replay example for gtsam::EqVIOFilter.

#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

class PinholeCamera final : public VIOCameraModel {
  double fx_, fy_, cx_, cy_;

 public:
  PinholeCamera(double fx, double fy, double cx, double cy)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

  Point2 projectPoint(const Point3& p) const override {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("PinholeCamera: z near zero");
    }
    return Point2(fx_ * p.x() / p.z() + cx_, fy_ * p.y() / p.z() + cy_);
  }

  Vector3 undistortPoint(const Point2& y) const override {
    return Vector3((y.x() - cx_) / fx_, (y.y() - cy_) / fy_, 1.0).normalized();
  }

  Matrix23 projectionJacobian(const Vector3& y) const override {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("PinholeCamera: z near zero");
    }
    Matrix23 J;
    const double z2 = y.z() * y.z();
    J << fx_ / y.z(), 0.0, -fx_ * y.x() / z2, 0.0, fy_ / y.z(),
        -fy_ * y.y() / z2;
    return J;
  }
};

void printUsage(const char* argv0) {
  std::cerr << "Usage:\n  " << argv0
            << " <bag> <imu_topic> <image_topic> <fx> <fy> <cx> <cy> [max_features]\n";
}

cv::Mat toGray(const sensor_msgs::ImageConstPtr& imageMsg) {
  try {
    cv_bridge::CvImageConstPtr cvPtr =
        cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::MONO8);
    return cvPtr->image;
  } catch (const cv_bridge::Exception&) {
    cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(imageMsg);
    cv::Mat gray;
    if (cvPtr->image.channels() == 1) {
      gray = cvPtr->image;
    } else {
      cv::cvtColor(cvPtr->image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
  }
}

bool insideImage(const cv::Point2f& p, int w, int h) {
  return p.x >= 0.0f && p.y >= 0.0f && p.x < static_cast<float>(w) &&
         p.y < static_cast<float>(h);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 8) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string bagPath = argv[1];
  const std::string imuTopic = argv[2];
  const std::string imageTopic = argv[3];
  const double fx = std::stod(argv[4]);
  const double fy = std::stod(argv[5]);
  const double cx = std::stod(argv[6]);
  const double cy = std::stod(argv[7]);
  const int maxFeatures = (argc > 8) ? std::stoi(argv[8]) : 200;

  EqVIOFilterParams params;
  params.coordinateChoice = CoordinateChoice::InvDepth;
  params.useDiscreteVelocityLift = true;
  params.useDiscreteInnovationLift = true;
  params.removeLostLandmarks = true;
  params.initialPointDepth = 8.0;
  params.initialPointVariance = 1.0;
  params.measurementNoiseVariance = 4.0;

  VIOSensorState sensor;
  sensor.inputBias = VIOBias::Identity();
  sensor.pose = Pose3::Identity();
  sensor.velocity.setZero();
  sensor.cameraOffset = Pose3::Identity();
  VIOState xi0(sensor, {});
  Matrix Sigma0 = Matrix::Identity(xi0.dim(), xi0.dim()) * 1e-3;
  EqVIOFilter filter(xi0, Sigma0, params, 0.0);
  auto camera = std::make_shared<PinholeCamera>(fx, fy, cx, cy);

  rosbag::Bag bag;
  bag.open(bagPath, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery({imuTopic, imageTopic}));

  cv::Mat prevGray;
  std::vector<cv::Point2f> prevPts;
  std::vector<int> prevIds;
  int nextId = 0;

  size_t imuCount = 0, imageCount = 0, visionCount = 0;

  for (const rosbag::MessageInstance& m : view) {
    if (m.getTopic() == imuTopic || ("/" + m.getTopic()) == imuTopic) {
      sensor_msgs::ImuConstPtr imuMsg = m.instantiate<sensor_msgs::Imu>();
      if (!imuMsg) continue;

      IMUInput imu;
      imu.stamp = imuMsg->header.stamp.toSec();
      imu.gyr = Vector3(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y,
                        imuMsg->angular_velocity.z);
      imu.acc = Vector3(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y,
                        imuMsg->linear_acceleration.z);
      filter.processIMUData(imu);
      ++imuCount;
      continue;
    }

    if (m.getTopic() == imageTopic || ("/" + m.getTopic()) == imageTopic) {
      sensor_msgs::ImageConstPtr imageMsg = m.instantiate<sensor_msgs::Image>();
      if (!imageMsg) continue;
      ++imageCount;

      cv::Mat gray = toGray(imageMsg);

      std::vector<cv::Point2f> trackedPts;
      std::vector<int> trackedIds;
      if (!prevGray.empty() && !prevPts.empty()) {
        std::vector<cv::Point2f> nextPts;
        std::vector<unsigned char> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prevGray, gray, prevPts, nextPts, status, err);
        for (size_t i = 0; i < status.size(); ++i) {
          if (!status[i]) continue;
          if (!insideImage(nextPts[i], gray.cols, gray.rows)) continue;
          trackedPts.push_back(nextPts[i]);
          trackedIds.push_back(prevIds[i]);
        }
      }

      if (static_cast<int>(trackedPts.size()) < maxFeatures) {
        std::vector<cv::Point2f> fresh;
        cv::goodFeaturesToTrack(gray, fresh, maxFeatures - trackedPts.size(), 0.01,
                                12.0);
        for (const cv::Point2f& p : fresh) {
          bool tooClose = false;
          for (const cv::Point2f& q : trackedPts) {
            const float dx = p.x - q.x;
            const float dy = p.y - q.y;
            if (dx * dx + dy * dy < 25.0f) {
              tooClose = true;
              break;
            }
          }
          if (tooClose) continue;
          trackedPts.push_back(p);
          trackedIds.push_back(nextId++);
          if (static_cast<int>(trackedPts.size()) >= maxFeatures) break;
        }
      }

      VisionMeasurement y;
      const double stamp = imageMsg->header.stamp.toSec();
      for (size_t i = 0; i < trackedPts.size(); ++i) {
        y[trackedIds[i]] = Point2(trackedPts[i].x, trackedPts[i].y);
      }

      if (!y.empty()) {
        filter.processVisionData(stamp, y, camera);
        ++visionCount;
      }

      prevGray = gray;
      prevPts = trackedPts;
      prevIds = trackedIds;
    }
  }

  bag.close();

  const VIOState est = filter.stateEstimate();
  std::cout << "Replay complete.\n";
  std::cout << "IMU: " << imuCount << ", images: " << imageCount
            << ", vision updates: " << visionCount << "\n";
  std::cout << "Filter time: " << filter.currentTime() << "\n";
  std::cout << "Landmarks: " << est.n() << "\n";
  std::cout << "Pose translation: " << est.sensor.pose.translation().transpose()
            << "\n";
  std::cout << "Velocity: " << est.sensor.velocity.transpose() << "\n";

  return 0;
}
