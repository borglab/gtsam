/**
 * @file PlanarGyroFactor.cpp
 * @author joel@truher.org
 * @date May 1, 2026
 */
#include <gtsam/navigation/PlanarGyroFactor.h>

namespace gtsam {
void PlanarGyroMeasurement::print(const std::string& s) const {
  std::cout << s;
  std::cout << " dt [" << deltaT_ << "]" << std::endl;
  std::cout << " dtheta = (" << deltaR_.theta() << ")" << std::endl;
}

bool PlanarGyroMeasurement::equals(const PlanarGyroMeasurement& other,
                                   double tol) const {
  return std::abs(ARW_ - other.ARW_) < tol &&
         deltaR_.equals(other.deltaR_, tol) &&
         std::abs(deltaT_ - other.deltaT_) < tol;
}

Rot2 PlanarGyroMeasurement::deltaR(double bias,
                                   OptionalJacobian<1, 1> H) const {
  if (H) (*H)(0) = -deltaT_;
  return deltaR_.compose(Rot2::fromAngle(-deltaT_ * bias));
}

Rot2 PlanarGyroMeasurement::predict(const Rot2& Ri, double bias,
                                    OptionalJacobian<1, 1> H1,
                                    OptionalJacobian<1, 1> H2) const {
  return Ri.compose(deltaR(bias, H2), H1);
}

double PlanarGyroMeasurement::computeError(const Rot2& Ri, const Rot2& Rj,
                                           double bias,
                                           OptionalJacobian<1, 1> H1,
                                           OptionalJacobian<1, 1> H2,
                                           OptionalJacobian<1, 1> H3) const {
  // Predict orientation at time j
  Matrix1 D_predict_Ri, D_predict_bias;
  Rot2 predicted_Rj = predict(Ri, bias, H1 ? &D_predict_Ri : nullptr,
                              H3 ? &D_predict_bias : nullptr);

  // Compute the error vector: log(Rj.inverse() * predicted_Rj)
  Matrix1 D_error_Rj, D_error_predict;
  Vector1 error = Rj.logmap(predicted_Rj, H2 ? &D_error_Rj : nullptr,
                            H1 || H3 ? &D_error_predict : nullptr);

  // Jacobians using the chain rule
  if (H1) *H1 = D_error_predict * D_predict_Ri;
  if (H2) *H2 = D_error_Rj;
  if (H3) *H3 = D_error_predict * D_predict_bias;

  return error(0);
}

PlanarGyroFactor::PlanarGyroFactor(Key pose_i, Key pose_j, Key bias,
                                   const PlanarGyroMeasurement& x)
    : Base(noiseModel::Constrained::MixedVariances(Vector3(0, 0, x.variance())),
           pose_i, pose_j, bias),
      measurement_(x) {}

gtsam::NonlinearFactor::shared_ptr PlanarGyroFactor::clone() const {
  return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PlanarGyroFactor(*this)));
}

void PlanarGyroFactor::print(const std::string& s,
                             const KeyFormatter& keyFormatter) const {
  std::cout << s << "PlanarGyroFactor("             //
            << keyFormatter(this->key<1>()) << ","  //
            << keyFormatter(this->key<2>()) << ","  //
            << keyFormatter(this->key<3>()) << ",";
  measurement_.print(" measurement:");
  noiseModel_->print(" noise model: ");
}

bool PlanarGyroFactor::equals(const NonlinearFactor& other, double tol) const {
  const PlanarGyroFactor* e = dynamic_cast<const PlanarGyroFactor*>(&other);
  return e != nullptr && Base::equals(*e, tol) &&
         measurement_.equals(e->measurement_, tol);
}

Vector PlanarGyroFactor::evaluateError(const Pose2& Pi, const Pose2& Pj,
                                       const double& bias,
                                       OptionalMatrixType H1,
                                       OptionalMatrixType H2,
                                       OptionalMatrixType H3) const {
  Matrix1 rH1, rH2, rH3;
  double err = measurement_.computeError(Pi.r(), Pj.r(), bias, H1 ? &rH1 : 0,
                                         H2 ? &rH2 : 0, H3 ? &rH3 : 0);
  if (H1) {
    *H1 = Z_3x3;
    H1->block<1, 1>(2, 2) = rH1;
  }
  if (H2) {
    *H2 = Z_3x3;
    H2->block<1, 1>(2, 2) = rH2;
  }
  if (H3) {
    *H3 = Z_3x1;
    H3->block<1, 1>(2, 0) = rH3;
  }
  return Vector3(0, 0, err);
}
}  // namespace gtsam