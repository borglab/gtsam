namespace gtsam {

#include <gtsam/geometry/Pose3.h>
class Pose3 {
  Pose3();
  Pose3(gtsam::Rot3 R, gtsam::Point3 t);

  gtsam::Pose3 operator*(gtsam::Pose3 other) const;
};

template<T = {Matrix}>
class Container {
  gtsam::JacobianFactor operator()(const T& m) const;
  T operator[](size_t idx) const;
};
}
