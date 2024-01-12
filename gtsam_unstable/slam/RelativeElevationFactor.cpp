/**
 * @file RelativeElevationFactor.cpp
 *
 * @date Aug 17, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/slam/RelativeElevationFactor.h>

namespace gtsam {

/* ************************************************************************* */
RelativeElevationFactor::RelativeElevationFactor(Key poseKey, Key pointKey, double measured,
    const SharedNoiseModel& model)
: Base(model, poseKey, pointKey), measured_(measured)
{
}

/* ************************************************************************* */
Vector RelativeElevationFactor::evaluateError(const Pose3& pose, const Point3& point,
    OptionalMatrixType H1, OptionalMatrixType H2) const {
  double hx = pose.z() - point.z();
  if (H1) {
    *H1 = Matrix::Zero(1,6);
    // Use bottom row of rotation matrix for derivative of translation
    (*H1)(0, 3) = pose.rotation().r1().z();
    (*H1)(0, 4) = pose.rotation().r2().z();
    (*H1)(0, 5) = pose.rotation().r3().z();
  }

  if (H2) {
    *H2 = Matrix::Zero(1,3);
    (*H2)(0, 2) = -1.0;
  }
  return (Vector(1) << hx - measured_).finished();
}

/* ************************************************************************* */
bool RelativeElevationFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This *e = dynamic_cast<const This*> (&expected);
  return e != nullptr && Base::equals(*e, tol) && std::abs(this->measured_ - e->measured_) < tol;
}

/* ************************************************************************* */
void RelativeElevationFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "RelativeElevationFactor, relative elevation = " << measured_ << std::endl;
  Base::print("", keyFormatter);
}
/* ************************************************************************* */

} // \namespace gtsam


