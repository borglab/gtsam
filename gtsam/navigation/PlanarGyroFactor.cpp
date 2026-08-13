/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarGyroFactor.cpp
 * @author joel@truher.org
 * @date May 1, 2026
 */
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/navigation/PlanarGyroFactor.h>

namespace gtsam {
using noiseModel::Constrained;
using noiseModel::Diagonal;

double PlanarGyroParams::arwSigma(double deltaT) { return arw * sqrt(deltaT); }

double PlanarGyroParams::biasInstabilitySigma() { return biasInstability; }

bool PlanarGyroParams::operator==(const PlanarGyroParams& other) const {
  return arw == other.arw && biasInstability == other.biasInstability;
}

void PlanarGyroParams::print(const std::string& s) const {
  std::cout << s                                               //
            << " arw [" << arw << "]"                          //
            << " biasInstability [" << biasInstability << ")"  //
            << std::endl;
}

PlanarGyroBiasFactor::PlanarGyroBiasFactor(
    Key bias_i, Key bias_j, const std::shared_ptr<PlanarGyroParams>& p)
    : Base(bias_i, bias_j, 0.0,
           Diagonal::Sigmas(Vector1(p->biasInstabilitySigma()))) {}

PlanarGyroFactor::PlanarGyroFactor(Key pose_i, Key pose_j, Key bias,
                                   const std::shared_ptr<PlanarGyroParams>& p,
                                   Rot2 dr, double dt)
    : Base(Constrained::MixedSigmas(Vector3(0, 0, p->arwSigma(dt))), pose_i,
           pose_j, bias),
      p_(p),
      deltaR_(dr),
      deltaT_(dt) {}

Rot2 PlanarGyroFactor::deltaR(double bias, OptionalJacobian<1, 1> H) const {
  if (H) (*H)(0) = -deltaT_;
  return deltaR_.compose(Rot2::fromAngle(-deltaT_ * bias));
}

Rot2 PlanarGyroFactor::predict(const Rot2& Ri, double bias,
                               OptionalJacobian<1, 1> H1,
                               OptionalJacobian<1, 1> H2) const {
  return Ri.compose(deltaR(bias, H2), H1);
}

double PlanarGyroFactor::computeError(const Rot2& Ri, const Rot2& Rj,
                                      double bias, OptionalJacobian<1, 1> H1,
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

gtsam::NonlinearFactor::shared_ptr PlanarGyroFactor::clone() const {
  return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PlanarGyroFactor(*this)));
}

void PlanarGyroFactor::print(const std::string& s,
                             const KeyFormatter& keyFormatter) const {
  std::cout << s << "PlanarGyroFactor("              //
            << keyFormatter(this->key<1>()) << ","   //
            << keyFormatter(this->key<2>()) << ","   //
            << keyFormatter(this->key<3>()) << ","   //
            << " dt [" << deltaT_ << "]" << ","      //
            << " dtheta [" << deltaR_.theta() << ""  //
            << std::endl;
  p_->print("params: ");
  noiseModel_->print(" noise model: ");
}

bool PlanarGyroFactor::equals(const NonlinearFactor& other, double tol) const {
  const PlanarGyroFactor* e = dynamic_cast<const PlanarGyroFactor*>(&other);
  return e != nullptr                        //
         && Base::equals(*e, tol)            //
         && p_ == e->p_                      //
         && deltaR_.equals(e->deltaR_, tol)  //
         && std::abs(deltaT_ - e->deltaT_) < tol;
}

Vector3 PlanarGyroFactor::evaluateError(const Pose2& Pi, const Pose2& Pj,
                                        const double& bias,
                                        OptionalMatrixType H1,
                                        OptionalMatrixType H2,
                                        OptionalMatrixType H3) const {
  Matrix1 rH1, rH2, rH3;
  double err = computeError(Pi.r(), Pj.r(), bias, H1 ? &rH1 : 0, H2 ? &rH2 : 0,
                            H3 ? &rH3 : 0);
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
