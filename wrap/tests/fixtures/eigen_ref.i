namespace gtsam {

#include <gtsam/geometry/Pose3.h>

class Pose3 {
  Pose3();

  // Case 1: two Ref args, primitive non-Ref input
  gtsam::Point3 transformFrom(const gtsam::Point3& point) const;
  gtsam::Point3 transformFrom(const gtsam::Point3& point,
                              Eigen::Ref<Eigen::MatrixXd> Hself,
                              Eigen::Ref<Eigen::MatrixXd> Hpoint) const;

  // Case 2: single Ref arg, no other inputs
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 inverse(Eigen::Ref<Eigen::MatrixXd> H) const;

  // Case 3: two Ref args, class-type non-Ref input
  gtsam::Pose3 between(const gtsam::Pose3& pose) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose,
                       Eigen::Ref<Eigen::MatrixXd> H1,
                       Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Case 4: static method with Ref arg
  static gtsam::Pose3 Expmap(gtsam::Vector xi);
  static gtsam::Pose3 Expmap(gtsam::Vector xi,
                              Eigen::Ref<Eigen::MatrixXd> Hxi);
};

}