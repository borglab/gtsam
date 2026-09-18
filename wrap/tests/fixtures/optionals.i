#include <folder/path/to/Pose3.h>

namespace gtsam {

class Pose3 {
  Pose3();

  std::optional<double> threshold;

  static std::optional<gtsam::Pose3> MaybePose(bool available);

  std::optional<gtsam::Vector> maybeVector(bool available) const;
  std::optional<std::pair<gtsam::Vector, gtsam::Matrix>> maybeGaussian(
      bool available) const;

  void acceptOptionalDouble(const std::optional<double>& value) const;
  void acceptOptionalPose(const std::optional<gtsam::Pose3>& value) const;
  void acceptOptionalVector(const std::optional<gtsam::Vector>& value) const;
};

}

std::optional<double> maybeDouble(bool available);
