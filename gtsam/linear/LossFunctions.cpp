/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NoiseModel.cpp
 * @date Jan 13, 2010
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#include <gtsam/3rdparty/cephes/cephes.h>
#include <gtsam/linear/LossFunctions.h>

#include <iostream>
#include <optional>
#include <utility>
#include <vector>

using namespace std;

namespace gtsam {
namespace noiseModel {

/* ************************************************************************* */
// M-Estimator
/* ************************************************************************* */

namespace mEstimator {

Vector Base::weight(const Vector& error) const {
  const size_t n = error.rows();
  Vector w(n);
  for (size_t i = 0; i < n; ++i)
    w(i) = weight(error(i));
  return w;
}

Vector Base::sqrtWeight(const Vector &error) const {
  return weight(error).cwiseSqrt();
}

// The following three functions re-weight block matrices and a vector
// according to their weight implementation

void Base::reweight(Vector& error) const {
  if (reweight_ == Block) {
    const double w = sqrtWeight(error.norm());
    error *= w;
  } else {
    error.array() *= weight(error).cwiseSqrt().array();
  }
}

// Reweight n block matrices with one error vector
void Base::reweight(vector<Matrix> &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    for(Matrix& Aj: A) {
      Aj *= w;
    }
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    for(Matrix& Aj: A) {
      vector_scale_inplace(W,Aj);
    }
    error = W.cwiseProduct(error);
  }
}

// Reweight one block matrix with one error vector
void Base::reweight(Matrix &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A);
    error = W.cwiseProduct(error);
  }
}

// Reweight two block matrix with one error vector
void Base::reweight(Matrix &A1, Matrix &A2, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A1 *= w;
    A2 *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A1);
    vector_scale_inplace(W,A2);
    error = W.cwiseProduct(error);
  }
}

// Reweight three block matrix with one error vector
void Base::reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A1 *= w;
    A2 *= w;
    A3 *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A1);
    vector_scale_inplace(W,A2);
    vector_scale_inplace(W,A3);
    error = W.cwiseProduct(error);
  }
}

/* ************************************************************************* */
// Null model
/* ************************************************************************* */

void Null::print(const std::string &s="") const
{ cout << s << "null ()" << endl; }

Null::shared_ptr Null::Create()
{ return shared_ptr(new Null()); }

/* ************************************************************************* */
// Fair
/* ************************************************************************* */

Fair::Fair(double c, const ReweightScheme reweight) : Base(reweight), c_(c) {
  if (c_ <= 0) {
    throw runtime_error("mEstimator Fair takes only positive double in constructor.");
  }
}

double Fair::weight(double distance) const {
  return 1.0 / (1.0 + std::abs(distance) / c_);
}

double Fair::loss(double distance) const {
  const double absError = std::abs(distance);
  const double normalizedError = absError / c_;
  const double c_2 = c_ * c_;
  return c_2 * (normalizedError - std::log1p(normalizedError));
}

double Fair::graduatedWeight(double /*error*/, double /*error*/) const {
  throw std::logic_error("Fair loss has no graduated form.");
}

double Fair::graduatedLoss(double /*error*/, double /*error*/) const {
  throw std::logic_error("Fair loss has no graduated form.");
}

void Fair::print(const std::string &s="") const
{ cout << s << "fair (" << c_ << ")" << endl; }

bool Fair::equals(const Base &expected, double tol) const {
  const Fair* p = dynamic_cast<const Fair*> (&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_ ) < tol;
}

Fair::shared_ptr Fair::Create(double c, const ReweightScheme reweight)
{ return shared_ptr(new Fair(c, reweight)); }

/* ************************************************************************* */
// Huber
/* ************************************************************************* */

Huber::Huber(double k, const ReweightScheme reweight) : Base(reweight), k_(k) {
  if (k_ <= 0) {
    throw runtime_error("mEstimator Huber takes only positive double in constructor.");
  }
}

double Huber::Weight(double k, double distance) {
  const double absError = std::abs(distance);
  return (absError <= k) ? (1.0) : (k / absError);
}

double Huber::Loss(double k, double distance) {
  const double absError = std::abs(distance);
  if (absError <= k) {  // |x| <= k
    return distance * distance / 2;
  } else {  // |x| > k
    return k * (absError - (k / 2));
  }
}

double Huber::weight(double distance) const { return Weight(k_, distance); }

double Huber::loss(double distance) const { return Loss(k_, distance); }

double Huber::graduatedWeight(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Weight(k_ * lambda, distance);
}

double Huber::graduatedLoss(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Loss(k_ * lambda, distance);
}

void Huber::print(const std::string &s="") const {
  cout << s << "huber (" << k_ << ")" << endl;
}

bool Huber::equals(const Base &expected, double tol) const {
  const Huber* p = dynamic_cast<const Huber*>(&expected);
  if (p == nullptr) return false;
  return std::abs(k_ - p->k_) < tol;
}

Huber::shared_ptr Huber::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Huber(c, reweight));
}

/* ************************************************************************* */
// Cauchy
/* ************************************************************************* */

Cauchy::Cauchy(double k, const ReweightScheme reweight) : Base(reweight), k_(k), ksquared_(k * k) {
  if (k <= 0) {
    throw runtime_error("mEstimator Cauchy takes only positive double in constructor.");
  }
}

double Cauchy::Weight(double ksquared, double distance) {
  return ksquared / (ksquared + distance * distance);
}

double Cauchy::Loss(double ksquared, double distance) {
  const double val = std::log1p(distance * distance / ksquared);
  return ksquared * val * 0.5;
}

double Cauchy::weight(double distance) const {
  return Weight(ksquared_, distance);
}

double Cauchy::loss(double distance) const { return Loss(ksquared_, distance); }

double Cauchy::graduatedWeight(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Cauchy::Weight(ksquared_ * (lambda * lambda), distance);
}

double Cauchy::graduatedLoss(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Cauchy::Loss(ksquared_ * (lambda * lambda), distance);
}

void Cauchy::print(const std::string &s="") const {
  cout << s << "cauchy (" << k_ << ")" << endl;
}

bool Cauchy::equals(const Base &expected, double tol) const {
  const Cauchy* p = dynamic_cast<const Cauchy*>(&expected);
  if (p == nullptr) return false;
  return std::abs(ksquared_ - p->ksquared_) < tol;
}

Cauchy::shared_ptr Cauchy::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Cauchy(c, reweight));
}

/* ************************************************************************* */
// Tukey
/* ************************************************************************* */

Tukey::Tukey(double c, const ReweightScheme reweight) : Base(reweight), c_(c), csquared_(c * c) {
  if (c <= 0) {
    throw runtime_error("mEstimator Tukey takes only positive double in constructor.");
  }
}

double Tukey::Weight(double c, double csquared, double distance) {
  if (std::abs(distance) <= c) {
    const double one_minus_xc2 = 1.0 - distance * distance / csquared;
    return one_minus_xc2 * one_minus_xc2;
  }
  return 0.0;
}

double Tukey::Loss(double c, double csquared, double distance) {
  const double absError = std::abs(distance);
  if (absError <= c) {
    const double one_minus_xc2 = 1.0 - distance * distance / csquared;
    const double t = one_minus_xc2 * one_minus_xc2 * one_minus_xc2;
    return csquared * (1 - t) / 6.0;
  } else {
    return csquared / 6.0;
  }
}

double Tukey::weight(double distance) const {
  return Weight(c_, csquared_, distance);
}

double Tukey::loss(double distance) const {
  return Loss(c_, csquared_, distance);
}

double Tukey::graduatedWeight(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Weight(c_ * lambda, csquared_ * (lambda * lambda), distance);
}

double Tukey::graduatedLoss(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Loss(c_ * lambda, csquared_ * (lambda * lambda), distance);
}

void Tukey::print(const std::string &s="") const {
  std::cout << s << ": Tukey (" << c_ << ")" << std::endl;
}

bool Tukey::equals(const Base &expected, double tol) const {
  const Tukey* p = dynamic_cast<const Tukey*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

Tukey::shared_ptr Tukey::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Tukey(c, reweight));
}

/* ************************************************************************* */
// Welsch
/* ************************************************************************* */

Welsch::Welsch(double c, const ReweightScheme reweight) : Base(reweight), c_(c), csquared_(c * c) {}

double Welsch::Weight(double csquared, double distance) {
  const double xc2 = (distance * distance) / csquared;
  return std::exp(-xc2);
}

double Welsch::Loss(double csquared, double distance) {
  const double xc2 = (distance * distance) / csquared;
  return csquared * 0.5 * -std::expm1(-xc2);
}

double Welsch::weight(double distance) const {
  return Weight(csquared_, distance);
}

double Welsch::loss(double distance) const { return Loss(csquared_, distance); }

double Welsch::graduatedWeight(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Weight(csquared_ * (lambda * lambda), distance);
}

double Welsch::graduatedLoss(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Loss(csquared_ * (lambda * lambda), distance);
}

void Welsch::print(const std::string &s="") const {
  std::cout << s << ": Welsch (" << c_ << ")" << std::endl;
}

bool Welsch::equals(const Base &expected, double tol) const {
  const Welsch* p = dynamic_cast<const Welsch*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

Welsch::shared_ptr Welsch::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Welsch(c, reweight));
}

/* ************************************************************************* */
// GemanMcClure
/* ************************************************************************* */
GemanMcClure::GemanMcClure(double c, const GemanMcClure::GradScheme graduation,
                           const ReweightScheme reweight)
    : Base(reweight), c_(c), csquared_(c * c), graduation_(graduation) {}

double GemanMcClure::Weight(double csquared, double distance2) {
  const double c4 = csquared * csquared;
  const double c2error = csquared + distance2;
  return c4 / (c2error * c2error);
}

double GemanMcClure::Loss(double csquared, double distance2) {
  return 0.5 * (csquared * distance2) / (csquared + distance2);
}

double GemanMcClure::weight(double distance) const {
  return Weight(csquared_, distance * distance);
}

double GemanMcClure::loss(double distance) const {
  return Loss(csquared_, distance * distance);
}

double GemanMcClure::graduatedWeight(double distance, double mu) const {
  const double d2 = distance * distance;
  if (graduation_ == GemanMcClure::GradScheme::STANDARD) {
    const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
    return Weight(csquared_ * (lambda * lambda), d2);
  } else {  // GemanMcClure::GradScheme::SCALE_INVARIANT
    const double sqrt_denom = csquared_ + std::pow(d2, mu);
    return (csquared_ * (csquared_ + std::pow(d2, mu) * (1 - mu))) /
           (sqrt_denom * sqrt_denom);
  }
}

double GemanMcClure::graduatedLoss(double distance, double mu) const {
  const double d2 = distance * distance;
  if (graduation_ == GemanMcClure::GradScheme::STANDARD) {
    const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
    return Loss(csquared_ * (lambda * lambda), d2);
  } else {  // GemanMcClure::GradScheme::SCALE_INVARIANT
    return 0.5 * (csquared_ * d2) / (csquared_ + std::pow(d2, mu));
  }
}

void GemanMcClure::print(const std::string &s="") const {
  std::cout << s << ": Geman-McClure (" << c_ << ")" << std::endl;
}

bool GemanMcClure::equals(const Base &expected, double tol) const {
  const GemanMcClure* p = dynamic_cast<const GemanMcClure*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

GemanMcClure::shared_ptr GemanMcClure::Create(
    double c, const GemanMcClure::GradScheme graduation,
    const ReweightScheme reweight) {
  return shared_ptr(new GemanMcClure(c, graduation, reweight));
}

/* ************************************************************************* */
double GemanMcClure::shapeParamFromInfThresh(double influence_thresh, size_t dof,
                                          double chi2_outlier_thresh) {
  double outlier_residual_thresh =
      2 * gtsam_cephes_igami(dof / 2, chi2_outlier_thresh);
  // Equation [d/dr \rho(x) = influence_thresh] solved for c
  const double t1 =
      std::sqrt(2 * influence_thresh * std::pow(outlier_residual_thresh, 5));
  const double t2 = influence_thresh * std::pow(outlier_residual_thresh, 2);
  const double t3 = influence_thresh - 2 * outlier_residual_thresh;
  return std::sqrt(-((t1 + t2) / t3));
}

/* ************************************************************************* */
// TruncatedLeastSquares
/* ************************************************************************* */

TruncatedLeastSquares::TruncatedLeastSquares(
    double c, const TruncatedLeastSquares::GradScheme graduation,
    const ReweightScheme reweight)
    : Base(reweight), c_(c), csquared_(c * c), graduation_(graduation) {
  if (c_ <= 0) {
    throw runtime_error(
        "mEstimator TruncatedLeastSquares takes only positive double in "
        "constructor.");
  }
}

double TruncatedLeastSquares::Weight(double csquared, double distance2) {
  if (distance2 <= csquared) {
    return 1.0;
  }
  return 0.0;
}

double TruncatedLeastSquares::Loss(double csquared, double distance2) {
  if (distance2 <= csquared) {
    return 0.5 * distance2;
  }
  return 0.5 * csquared;
}

double TruncatedLeastSquares::GraduatedWeight(GradScheme graduation,
                                              double csquared, double distance2,
                                              double mu) {
  if (graduation == TruncatedLeastSquares::GradScheme::STANDARD) {
    const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
    return Weight(csquared * (lambda * lambda), distance2);
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_LINEAR) {
    const double lambda = std::clamp(mu, 0.0, 1.0) * lambda_max_ + lambda_min_;
    const double lowerbound = (lambda / (lambda + 1.0)) * csquared;
    const double upperbound = ((lambda + 1.0) / lambda) * csquared;

    if (distance2 <= lowerbound) return 1.0;
    if (distance2 >= upperbound) return 0.0;
    return std::sqrt(csquared * lambda * (lambda + 1.0) / distance2) - lambda;
  } else {  // TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR
    const double lambda = std::clamp(mu, 0.0, 1.0) * lambda_max_ + lambda_min_;
    const double lowerbound = csquared;
    const double upperbound = ((lambda + 1.0) * (lambda + 1.0) / (lambda * lambda)) * csquared;

    if (distance2 <= lowerbound) return 1.0;
    if (distance2 >= upperbound) return 0.0;
    return std::sqrt(csquared / distance2) * (lambda + 1.0) - lambda;
  }
}

double TruncatedLeastSquares::GraduatedLoss(GradScheme graduation,
                                            double csquared, double distance2,
                                            double mu) {
  if (graduation == TruncatedLeastSquares::GradScheme::STANDARD) {
    const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
    return Loss(csquared * (lambda * lambda), distance2);
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_LINEAR) {
    const double lambda = std::clamp(mu, 0.0, 1.0) * lambda_max_ + lambda_min_;
    const double lowerbound = (lambda / (lambda + 1.0)) * csquared;
    const double upperbound = ((lambda + 1.0) / lambda) * csquared;

    if (distance2 <= lowerbound) return 0.5 * distance2;
    if (distance2 >= upperbound) return 0.5 * csquared;
    return 0.5 * (2 * std::sqrt(csquared) * std::sqrt(distance2) *
                      std::sqrt(lambda * (lambda + 1.0)) -
                  (lambda * (csquared + distance2)));
  } else {  // TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR
    throw std::runtime_error(
        "TLS with GradScheme::GNC_SUPERLINEAR has no loss form");
  }
}

double TruncatedLeastSquares::weight(double distance) const {
  return Weight(csquared_, distance * distance);
}

double TruncatedLeastSquares::loss(double distance) const {
  return Loss(csquared_, distance * distance);
}

double TruncatedLeastSquares::graduatedWeight(double distance,
                                              double mu) const {
  return GraduatedWeight(graduation_, csquared_, distance * distance, mu);
}

double TruncatedLeastSquares::graduatedLoss(double distance, double mu) const {
  return GraduatedLoss(graduation_, csquared_, distance * distance, mu);
}

void TruncatedLeastSquares::print(const std::string& s = "") const {
  std::cout << s << ": TLS (" << c_ << ")" << std::endl;
}

bool TruncatedLeastSquares::equals(const Base& expected, double tol) const {
  const TruncatedLeastSquares* p =
      dynamic_cast<const TruncatedLeastSquares*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

TruncatedLeastSquares::shared_ptr TruncatedLeastSquares::Create(
    double c, TruncatedLeastSquares::GradScheme graduation,
    const ReweightScheme reweight) {
  return shared_ptr(new TruncatedLeastSquares(c, graduation, reweight));
}

/* ************************************************************************* */
// DCS
/* ************************************************************************* */
DCS::DCS(double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
}

double DCS::Weight(double c, double distance) {
  const double e2 = distance * distance;
  if (e2 > c) {
    const double w = 2.0 * c / (c + e2);
    return w * w;
  }
  return 1.0;
}

double DCS::Loss(double c, double distance) {
  // This is the simplified version of Eq 9 from (Agarwal13icra)
  // after you simplify and cancel terms.
  const double e2 = distance * distance;
  const double e4 = e2 * e2;
  const double c2 = c * c;
  return (c2 * e2 + c * e4) / ((e2 + c) * (e2 + c));
}

double DCS::weight(double distance) const { return Weight(c_, distance); }

double DCS::loss(double distance) const { return Loss(c_, distance); }

double DCS::graduatedWeight(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Weight(c_ * lambda, distance);
}

double DCS::graduatedLoss(double distance, double mu) const {
  const double lambda = 1.0 + ((1.0 - std::clamp(mu, 0.0, 1.0)) * lambda_max_);
  return Loss(c_ * lambda, distance);
}

void DCS::print(const std::string &s="") const {
  std::cout << s << ": DCS (" << c_ << ")" << std::endl;
}

bool DCS::equals(const Base &expected, double tol) const {
  const DCS* p = dynamic_cast<const DCS*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

DCS::shared_ptr DCS::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new DCS(c, reweight));
}

/* ************************************************************************* */
// L2WithDeadZone
/* ************************************************************************* */

L2WithDeadZone::L2WithDeadZone(double k, const ReweightScheme reweight)
 : Base(reweight), k_(k) {
  if (k_ <= 0) {
    throw runtime_error("mEstimator L2WithDeadZone takes only positive double in constructor.");
  }
}

double L2WithDeadZone::weight(double distance) const {
  // note that this code is slightly uglier than residual, because there are three distinct
  // cases to handle (left of deadzone, deadzone, right of deadzone) instead of the two
  // cases (deadzone, non-deadzone) in residual.
  if (std::abs(distance) <= k_) return 0.0;
  else if (distance > k_) return (-k_+distance)/distance;
  else return (k_+distance)/distance;
}

double L2WithDeadZone::loss(double distance) const {
  const double abs_error = std::abs(distance);
  return (abs_error < k_) ? 0.0 : 0.5*(k_-abs_error)*(k_-abs_error);
}

double L2WithDeadZone::graduatedWeight(double /*error*/,
                                       double /*error*/) const {
  throw std::logic_error("L2WithDeadZone loss has no graduated form.");
}

double L2WithDeadZone::graduatedLoss(double /*error*/, double /*error*/) const {
  throw std::logic_error("L2WithDeadZone loss has no graduated form.");
}

void L2WithDeadZone::print(const std::string &s="") const {
  std::cout << s << ": L2WithDeadZone (" << k_ << ")" << std::endl;
}

bool L2WithDeadZone::equals(const Base &expected, double tol) const {
  const L2WithDeadZone* p = dynamic_cast<const L2WithDeadZone*>(&expected);
  if (p == nullptr) return false;
  return std::abs(k_ - p->k_) < tol;
}

L2WithDeadZone::shared_ptr L2WithDeadZone::Create(double k, const ReweightScheme reweight) {
  return shared_ptr(new L2WithDeadZone(k, reweight));
}


/* ************************************************************************* */
// AsymmetricTukey
/* ************************************************************************* */

AsymmetricTukey::AsymmetricTukey(double c, const ReweightScheme reweight) : Base(reweight), c_(c), csquared_(c * c) {
  if (c <= 0) {
    throw runtime_error("mEstimator AsymmetricTukey takes only positive double in constructor.");
  }
}

double AsymmetricTukey::weight(double distance) const {
  distance = -distance;
  if (distance >= 0.0) {
    return 1.0;
  } else if (distance > -c_) {
    const double one_minus_xc2 = 1.0 - distance * distance / csquared_;
    return one_minus_xc2 * one_minus_xc2;
  }
  return 0.0;
}

double AsymmetricTukey::loss(double distance) const {
  distance = -distance;
  if (distance >= 0.0) {
    return distance * distance / 2.0;
  } else if (distance >= -c_) {
    const double one_minus_xc2 = 1.0 - distance * distance / csquared_;
    const double t = one_minus_xc2 * one_minus_xc2 * one_minus_xc2;
    return csquared_ * (1 - t) / 6.0;
  }
  return csquared_ / 6.0;
}

double AsymmetricTukey::graduatedWeight(double /*error*/,
                                        double /*error*/) const {
  throw std::logic_error("AsymmetricTukey loss has no graduated form.");
}

double AsymmetricTukey::graduatedLoss(double /*error*/,
                                      double /*error*/) const {
  throw std::logic_error("AsymmetricTukey loss has no graduated form.");
}

void AsymmetricTukey::print(const std::string &s="") const {
  std::cout << s << ": AsymmetricTukey (" << c_ << ")" << std::endl;
}

bool AsymmetricTukey::equals(const Base &expected, double tol) const {
  const AsymmetricTukey* p = dynamic_cast<const AsymmetricTukey*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

AsymmetricTukey::shared_ptr AsymmetricTukey::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new AsymmetricTukey(c, reweight));
}


/* ************************************************************************* */
// AsymmetricCauchy
/* ************************************************************************* */

AsymmetricCauchy::AsymmetricCauchy(double k, const ReweightScheme reweight) : Base(reweight), k_(k), ksquared_(k * k) {
  if (k <= 0) {
    throw runtime_error("mEstimator AsymmetricCauchy takes only positive double in constructor.");
  }
}

double AsymmetricCauchy::weight(double distance) const {
  distance = -distance;
  if (distance >= 0.0) {
    return 1.0;
  }
  
    return ksquared_ / (ksquared_ + distance*distance);
  
}

double AsymmetricCauchy::loss(double distance) const {
  distance = -distance;
  if (distance >= 0.0) {
    return distance * distance / 2.0;
  }
  const double val = std::log1p(distance * distance / ksquared_);
  return ksquared_ * val * 0.5;
}

double AsymmetricCauchy::graduatedWeight(double /*error*/,
                                         double /*error*/) const {
  throw std::logic_error("AsymmetricCauchy loss has no graduated form.");
}

double AsymmetricCauchy::graduatedLoss(double /*error*/,
                                       double /*error*/) const {
  throw std::logic_error("AsymmetricCauchy loss has no graduated form.");
}

void AsymmetricCauchy::print(const std::string &s="") const {
  std::cout << s << ": AsymmetricCauchy (" << k_ << ")" << std::endl;
}

bool AsymmetricCauchy::equals(const Base &expected, double tol) const {
  const AsymmetricCauchy* p = dynamic_cast<const AsymmetricCauchy*>(&expected);
  if (p == nullptr) return false;
  return std::abs(k_ - p->k_) < tol;
}

AsymmetricCauchy::shared_ptr AsymmetricCauchy::Create(double k, const ReweightScheme reweight) {
  return shared_ptr(new AsymmetricCauchy(k, reweight));
}


/* ************************************************************************* */
// Custom
/* ************************************************************************* */

Custom::Custom(std::function<double(double)> weight,
               std::function<double(double)> loss,
               std::optional<std::function<double(double, double)>> grad_weight,
               std::optional<std::function<double(double, double)>> grad_loss,
               const ReweightScheme reweight, std::string name)
    : Base(reweight),
      weight_(std::move(weight)),
      loss_(loss),
      grad_weight_(grad_weight),
      grad_loss_(grad_loss),
      name_(std::move(name)) {}

double Custom::weight(double distance) const { return weight_(distance); }

double Custom::loss(double distance) const { return loss_(distance); }

double Custom::graduatedWeight(double distance, double mu) const {
  if (grad_weight_) {
    return (*grad_weight_)(distance, mu);
  } else {
    throw std::logic_error("Custom loss provided no graduated form.");
  }
}

double Custom::graduatedLoss(double distance, double mu) const {
  if (grad_loss_) {
    return (*grad_loss_)(distance, mu);
  } else {
    throw std::logic_error("Custom loss provided no graduated form.");
  }
}

void Custom::print(const std::string &s = "") const {
  std::cout << s << ": Custom (" << name_ << ")" << std::endl;
}

bool Custom::equals(const Base &expected, double tol) const {
  const auto *p = dynamic_cast<const Custom *>(&expected);
  if (p == nullptr)
    return false;
  return name_ == p->name_ && weight_.target<double(double)>() == p->weight_.target<double(double)>() &&
         loss_.target<double(double)>() == p->loss_.target<double(double)>() && reweight_ == p->reweight_;
}

Custom::shared_ptr Custom::Create(
    std::function<double(double)> weight, std::function<double(double)> loss,
    std::optional<std::function<double(double, double)>> grad_weight,
    std::optional<std::function<double(double, double)>> grad_loss,
    const ReweightScheme reweight, const std::string& name) {
  return std::make_shared<Custom>(std::move(weight), std::move(loss),
                                  std::move(grad_weight), std::move(grad_loss),
                                  reweight, name);
}

} // namespace mEstimator
} // namespace noiseModel
} // gtsam
