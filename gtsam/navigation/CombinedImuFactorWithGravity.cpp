/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactorWithGravity.cpp
 *  @author Nikhil Khedekar
 **/

#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/LieGroupPreintegration.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <ostream>

namespace gtsam {

using namespace std;

//------------------------------------------------------------------------------
// CombinedImuFactorWithGravityT methods
//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
std::ostream& operator<<(std::ostream& os,
                         const CombinedImuFactorWithGravityT<PIM, GRAVITY>& f) {
  f.preintegratedMeasurements().print("combined preintegrated measurements:\n");
  if (internal::GravityParametrization<GRAVITY>::usesMagnitude)
    os << "  gravity magnitude: " << f.gravityMagnitude() << "\n";
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
void CombinedImuFactorWithGravityT<PIM, GRAVITY>::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? s : s + "\n") << "CombinedImuFactorWithGravity("
       << keyFormatter(this->template key<1>()) << "," << keyFormatter(this->template key<2>())
       << "," << keyFormatter(this->template key<3>()) << "," << keyFormatter(this->template key<4>())
       << "," << keyFormatter(this->template key<5>()) << "," << keyFormatter(this->template key<6>())
       << "," << keyFormatter(this->template key<7>()) << ")\n";
  std::cout << *this << std::endl;
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
bool CombinedImuFactorWithGravityT<PIM, GRAVITY>::equals(
    const NonlinearFactor& other, double tol) const {
  const This* e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && pim_.equals(e->pim_, tol) &&
         (!internal::GravityParametrization<GRAVITY>::usesMagnitude ||
          std::abs(gravityMagnitude_ - e->gravityMagnitude_) < tol);
}

//------------------------------------------------------------------------------
template <class PIM, class GRAVITY>
Vector CombinedImuFactorWithGravityT<PIM, GRAVITY>::evaluateError(
    const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j,
    const Vector3& vel_j, const imuBias::ConstantBias& bias_i,
    const imuBias::ConstantBias& bias_j, const GRAVITY& gravity,
    OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4, OptionalMatrixType H5, OptionalMatrixType H6,
    OptionalMatrixType H7) const {
  typedef internal::GravityParametrization<GRAVITY> Parametrization;

  Eigen::Matrix<double, 3, Parametrization::dimension> D_gvec_gravity;
  const Vector3 n_gravity = Parametrization::vector(
      gravity, gravityMagnitude_, H7 ? &D_gvec_gravity : nullptr);

  Matrix93 D_r_gvec;
  const Vector r = internal::combinedImuError(
      pim_, pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, n_gravity,
      H1, H2, H3, H4, H5, H6, H7 ? &D_r_gvec : nullptr);

  if (H7) {
    // Only the preintegration rows depend on gravity:
    H7->resize(15, Parametrization::dimension);
    H7->template block<9, Parametrization::dimension>(0, 0) =
        D_r_gvec * D_gvec_gravity;
    H7->template block<6, Parametrization::dimension>(9, 0).setZero();
  }
  return r;
}

//------------------------------------------------------------------------------
// CombinedImuFactorWithGravityT instantiations
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>, Unit3>;
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<TangentPreintegration>, Unit3>;
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<
    PreintegratedCombinedMeasurementsT<LieGroupPreintegration>, Unit3>;
template class GTSAM_EXPORT
    CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsG, Unit3>;
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>, Point3>;
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<TangentPreintegration>, Point3>;
template class GTSAM_EXPORT CombinedImuFactorWithGravityT<
    PreintegratedCombinedMeasurementsT<LieGroupPreintegration>, Point3>;
template class GTSAM_EXPORT
    CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsG, Point3>;

template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<TangentPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const CombinedImuFactorWithGravityT<
        PreintegratedCombinedMeasurementsT<LieGroupPreintegration>, Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsG,
                                        Unit3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<ManifoldPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os, const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsT<TangentPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const CombinedImuFactorWithGravityT<
        PreintegratedCombinedMeasurementsT<LieGroupPreintegration>, Point3>& f);
template GTSAM_EXPORT std::ostream& operator<<(
    std::ostream& os,
    const CombinedImuFactorWithGravityT<PreintegratedCombinedMeasurementsG,
                                        Point3>& f);

}  // namespace gtsam
