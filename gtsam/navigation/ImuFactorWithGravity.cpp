/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ImuFactorWithGravity.cpp
 *  @author Nikhil Khedekar
 **/

#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/ImuFactorWithGravity.h>
#include <gtsam/navigation/LieGroupPreintegration.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// ImuFactorWithGravityT methods
//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
std::ostream& operator<<(std::ostream& os, const ImuFactorWithGravityT<PIM, GRAVITY>& f) {
  f.preintegratedMeasurements().print("preintegrated measurements:\n");
  if (internal::GravityParametrization<GRAVITY>::usesMagnitude)
    os << "  gravity magnitude: " << f.gravityMagnitude() << "\n";
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
void ImuFactorWithGravityT<PIM, GRAVITY>::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactorWithGravity("
       << keyFormatter(this->template key<1>()) << "," << keyFormatter(this->template key<2>())
       << "," << keyFormatter(this->template key<3>()) << "," << keyFormatter(this->template key<4>())
       << "," << keyFormatter(this->template key<5>()) << "," << keyFormatter(this->template key<6>())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
bool ImuFactorWithGravityT<PIM, GRAVITY>::equals(const NonlinearFactor& other,
    double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && pim_.equals(e->pim_, tol) &&
         (!internal::GravityParametrization<GRAVITY>::usesMagnitude ||
          std::abs(gravityMagnitude_ - e->gravityMagnitude_) < tol);
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
Vector ImuFactorWithGravityT<PIM, GRAVITY>::evaluateError(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, const GRAVITY& gravity,
    OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4, OptionalMatrixType H5, OptionalMatrixType H6) const {
  typedef internal::GravityParametrization<GRAVITY> Parametrization;
  Eigen::Matrix<double, 3, Parametrization::dimension> D_gvec_gravity;
  const Vector3 n_gravity = Parametrization::vector(
      gravity, gravityMagnitude_, H6 ? &D_gvec_gravity : nullptr);
  Matrix93 D_r_gvec;
  const Vector r = internal::preintegrationErrorAndJacobians(
      pim_, pose_i, vel_i, pose_j, vel_j, bias_i, n_gravity, H1, H2, H3, H4,
      H5, H6 ? &D_r_gvec : nullptr);
  if (H6) *H6 = D_r_gvec * D_gvec_gravity;
  return r;
}

//------------------------------------------------------------------------------
// ImuFactor2WithGravityT methods
//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
std::ostream& operator<<(std::ostream& os,
                         const ImuFactor2WithGravityT<PIM, GRAVITY>& f) {
  f.preintegratedMeasurements().print("preintegrated measurements:\n");
  if (internal::GravityParametrization<GRAVITY>::usesMagnitude)
    os << "  gravity magnitude: " << f.gravityMagnitude() << "\n";
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
void ImuFactor2WithGravityT<PIM, GRAVITY>::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? s : s + "\n") << "ImuFactor2WithGravity("
       << keyFormatter(this->template key<1>()) << ","
       << keyFormatter(this->template key<2>()) << ","
       << keyFormatter(this->template key<3>()) << ","
       << keyFormatter(this->template key<4>()) << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
bool ImuFactor2WithGravityT<PIM, GRAVITY>::equals(const NonlinearFactor& other,
    double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && pim_.equals(e->pim_, tol) &&
         (!internal::GravityParametrization<GRAVITY>::usesMagnitude ||
          std::abs(gravityMagnitude_ - e->gravityMagnitude_) < tol);
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
Vector9 ImuFactor2WithGravityT<PIM, GRAVITY>::evaluateError(
    const NavState& state_i, const NavState& state_j,
    const imuBias::ConstantBias& bias_i, const GRAVITY& gravity,
    OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4) const {
  typedef internal::GravityParametrization<GRAVITY> Parametrization;
  Eigen::Matrix<double, 3, Parametrization::dimension> D_gvec_gravity;
  const Vector3 n_gravity = Parametrization::vector(
      gravity, gravityMagnitude_, H4 ? &D_gvec_gravity : nullptr);
  Matrix93 D_r_gvec;
  const Vector9 r =
      internal::preintegrationError(pim_, state_i, state_j, bias_i, n_gravity,
                                    H1, H2, H3, H4 ? &D_r_gvec : nullptr);
  if (H4) *H4 = D_r_gvec * D_gvec_gravity;
  return r;
}

//------------------------------------------------------------------------------
// ImuFactorWithGravityT instantiations
template class GTSAM_EXPORT ImuFactorWithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Unit3>;
template class GTSAM_EXPORT ImuFactorWithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Unit3>;
template class GTSAM_EXPORT ImuFactorWithGravityT<
    PreintegratedImuMeasurementsT<LieGroupPreintegration>, Unit3>;
template class GTSAM_EXPORT
    ImuFactorWithGravityT<PreintegratedImuMeasurementsG, Unit3>;
template class GTSAM_EXPORT ImuFactorWithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Point3>;
template class GTSAM_EXPORT ImuFactorWithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Point3>;
template class GTSAM_EXPORT ImuFactorWithGravityT<
    PreintegratedImuMeasurementsT<LieGroupPreintegration>, Point3>;
template class GTSAM_EXPORT
    ImuFactorWithGravityT<PreintegratedImuMeasurementsG, Point3>;

template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactorWithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactorWithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactorWithGravityT<
        PreintegratedImuMeasurementsT<LieGroupPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactorWithGravityT<PreintegratedImuMeasurementsG, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactorWithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactorWithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactorWithGravityT<
        PreintegratedImuMeasurementsT<LieGroupPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactorWithGravityT<PreintegratedImuMeasurementsG, Point3>& f);

//------------------------------------------------------------------------------
// ImuFactor2WithGravityT instantiations
template class GTSAM_EXPORT ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Unit3>;
template class GTSAM_EXPORT ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Unit3>;
template class GTSAM_EXPORT ImuFactor2WithGravityT<
    PreintegratedImuMeasurementsT<LieGroupPreintegration>, Unit3>;
template class GTSAM_EXPORT
    ImuFactor2WithGravityT<PreintegratedImuMeasurementsG, Unit3>;
template class GTSAM_EXPORT ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Point3>;
template class GTSAM_EXPORT ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Point3>;
template class GTSAM_EXPORT ImuFactor2WithGravityT<
    PreintegratedImuMeasurementsT<LieGroupPreintegration>, Point3>;
template class GTSAM_EXPORT
    ImuFactor2WithGravityT<PreintegratedImuMeasurementsG, Point3>;

template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactor2WithGravityT<
        PreintegratedImuMeasurementsT<LieGroupPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactor2WithGravityT<PreintegratedImuMeasurementsG, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<ManifoldPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const ImuFactor2WithGravityT<PreintegratedImuMeasurementsT<TangentPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactor2WithGravityT<
        PreintegratedImuMeasurementsT<LieGroupPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const ImuFactor2WithGravityT<PreintegratedImuMeasurementsG, Point3>& f);

}  // namespace gtsam
