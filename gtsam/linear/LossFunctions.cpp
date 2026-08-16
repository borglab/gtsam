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

#include <algorithm>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std;

namespace gtsam {
namespace noiseModel {

/* ************************************************************************* */
// M-Estimator
/* ************************************************************************* */

namespace mEstimator {

/** Helpers for Graduated Robust Losses
 * GTSAM convention for graduated robust losses:
 *   mu in [0, 1]
 *   mu = 0: most convex / least-squares-like
 *   mu = 1: final target robust loss
 * Increasing mu always increases non-convexity.
 *
 * Historical (orig. pub.) Yang-GM mu is replaced by lambda = 1 / mu.
 * Historical (orig. pub.) Yang/Peng TLS mu is replaced by theta = mu / (1 - mu)
 *
 * Losses that graduate by scaling their shape parameter do so with the exact
 * map lambda = 1 / mu, edge case mu = 0 and mu = 1 are handled with
 * direct formulas so that no unbounded quantity is ever evaluated.
 */
namespace {
/// @brief Restrict a public graduation parameter to its valid range [0, 1].
double ClampMu(double mu) { return std::clamp(mu, 0.0, 1.0); }

/// @brief Graduate shape parameter by lambda = 1 / mu.
/// Requires mu > 0; callers handle mu = 0 with the least-squares endpoint.
double GradShapeSquared(double csquared, double mu) { return csquared / mu; }

/// @brief Graduate a shape parameter by sqrt(1 / mu).
/// Requires mu > 0; callers handle mu = 0 with the least-squares endpoint.
double GradShape(double c, double mu) { return c / std::sqrt(mu); }
}  // namespace

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

double Huber::Weight(double distance, double k) {
  const double absError = std::abs(distance);
  return (absError <= k) ? (1.0) : (k / absError);
}

double Huber::Loss(double distance, double k) {
  const double absError = std::abs(distance);
  if (absError <= k) {  // |x| <= k
    return distance * distance / 2;
  } else {  // |x| > k
    return k * (absError - (k / 2));
  }
}

double Huber::weight(double distance) const { return Weight(distance, k_); }

double Huber::loss(double distance) const { return Loss(distance, k_); }

double Huber::graduatedWeight(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 1.0;
  return Weight(distance, GradShape(k_, m));
}

double Huber::graduatedLoss(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 0.5 * distance * distance;
  return Loss(distance, GradShape(k_, m));
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

double Cauchy::Weight(double distance, double ksquared) {
  return ksquared / (ksquared + distance * distance);
}

double Cauchy::Loss(double distance, double ksquared) {
  const double val = std::log1p(distance * distance / ksquared);
  return ksquared * val * 0.5;
}

double Cauchy::weight(double distance) const {
  return Weight(distance, ksquared_);
}

double Cauchy::loss(double distance) const { return Loss(distance, ksquared_); }

double Cauchy::graduatedWeight(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 1.0;
  return Cauchy::Weight(distance, GradShapeSquared(ksquared_, m));
}

double Cauchy::graduatedLoss(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 0.5 * distance * distance;
  return Cauchy::Loss(distance, GradShapeSquared(ksquared_, m));
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

double Tukey::Weight(double distance, double c, double csquared) {
  if (std::abs(distance) <= c) {
    const double one_minus_xc2 = 1.0 - distance * distance / csquared;
    return one_minus_xc2 * one_minus_xc2;
  }
  return 0.0;
}

double Tukey::Loss(double distance, double c, double csquared) {
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
  return Weight(distance, c_, csquared_);
}

double Tukey::loss(double distance) const {
  return Loss(distance, c_, csquared_);
}

double Tukey::graduatedWeight(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 1.0;
  return Weight(distance, GradShape(c_, m), GradShapeSquared(csquared_, m));
}

double Tukey::graduatedLoss(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 0.5 * distance * distance;
  return Loss(distance, GradShape(c_, m), GradShapeSquared(csquared_, m));
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

double Welsch::Weight(double distance, double csquared) {
  const double xc2 = (distance * distance) / csquared;
  return std::exp(-xc2);
}

double Welsch::Loss(double distance, double csquared) {
  const double xc2 = (distance * distance) / csquared;
  return csquared * 0.5 * -std::expm1(-xc2);
}

double Welsch::weight(double distance) const {
  return Weight(distance, csquared_);
}

double Welsch::loss(double distance) const { return Loss(distance, csquared_); }

double Welsch::graduatedWeight(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 1.0;
  return Weight(distance, GradShapeSquared(csquared_, m));
}

double Welsch::graduatedLoss(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 0.5 * distance * distance;
  return Loss(distance, GradShapeSquared(csquared_, m));
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
GemanMcClure::GemanMcClure(double c, const ReweightScheme reweight)
    : GemanMcClure(c, GradScheme::STANDARD, reweight) {}

GemanMcClure::GemanMcClure(double c, const GemanMcClure::GradScheme graduation,
                           const ReweightScheme reweight)
    : Base(reweight), c_(c), csquared_(c * c), graduation_(graduation) {}

double GemanMcClure::Weight(double distance2, double c2) {
  const double c4 = c2 * c2;
  const double c2error = c2 + distance2;
  return c4 / (c2error * c2error);
}

double GemanMcClure::Loss(double distance2, double c2) {
  return 0.5 * (c2 * distance2) / (c2 + distance2);
}

double GemanMcClure::GraduatedWeight(double distance2, double c2, double mu,
                                     GradScheme graduation) {
  const double m = ClampMu(mu);
  if (graduation == GemanMcClure::GradScheme::STANDARD) {
    // Normalized Yang-GM
    if (m <= 0.0) return 1.0;
    return Weight(distance2, GradShapeSquared(c2, m));
  } else {  // GemanMcClure::GradScheme::SCALE_INVARIANT
    const double d2mu = std::pow(distance2, m);
    const double sqrtDenominator = c2 + d2mu;
    return (c2 * (c2 + d2mu * (1 - m))) / (sqrtDenominator * sqrtDenominator);
  }
}

double GemanMcClure::GraduatedLoss(double distance2, double c2, double mu,
                                   GradScheme graduation) {
  const double m = ClampMu(mu);
  if (graduation == GemanMcClure::GradScheme::STANDARD) {
    // Normalized Yang-GM, see GraduatedWeight.
    if (m <= 0.0) return 0.5 * distance2;
    return Loss(distance2, GradShapeSquared(c2, m));
  } else {  // GemanMcClure::GradScheme::SCALE_INVARIANT
    return 0.5 * (c2 * distance2) / (c2 + std::pow(distance2, m));
  }
}

double GemanMcClure::weight(double distance) const {
  return Weight(distance * distance, csquared_);
}

double GemanMcClure::loss(double distance) const {
  return Loss(distance * distance, csquared_);
}

double GemanMcClure::graduatedWeight(double distance, double mu) const {
  return GraduatedWeight(distance * distance, csquared_, mu, graduation_);
}

double GemanMcClure::graduatedLoss(double distance, double mu) const {
  return GraduatedLoss(distance * distance, csquared_, mu, graduation_);
}

static std::string GemanMcClureGradSchemeName(
    GemanMcClure::GradScheme graduation) {
  if (graduation == GemanMcClure::GradScheme::STANDARD) {
    return "STANDARD";
  } else if (graduation == GemanMcClure::GradScheme::SCALE_INVARIANT) {
    return "SCALE_INVARIANT";
  } else {
    throw std::runtime_error("Invalid GemanMcClure::GradScheme");
  }
}

void GemanMcClure::print(const std::string& s = "") const {
  std::cout << s << ": Geman-McClure (" << c_ << ", "
            << GemanMcClureGradSchemeName(graduation_) << ")" << std::endl;
}

bool GemanMcClure::equals(const Base& expected, double tol) const {
  const GemanMcClure* p = dynamic_cast<const GemanMcClure*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol && graduation_ == p->graduation_;
}

GemanMcClure::shared_ptr GemanMcClure::Create(double c,
                                              const ReweightScheme reweight) {
  return shared_ptr(new GemanMcClure(c, reweight));
}

GemanMcClure::shared_ptr GemanMcClure::Create(
    double c, const GemanMcClure::GradScheme graduation,
    const ReweightScheme reweight) {
  return shared_ptr(new GemanMcClure(c, graduation, reweight));
}

/* ************************************************************************* */
double GemanMcClure::ShapeParameterFromInfluenceThreshold(
    double influenceThreshold, size_t dof, double chiSquaredOutlierThreshold) {
  const double r =
      std::sqrt(2 * gtsam_cephes_igami(dof / 2.0, chiSquaredOutlierThreshold));
  if (!(influenceThreshold > 0.0 && influenceThreshold < r)) {
    throw std::invalid_argument(
        "GemanMcClure::ShapeParameterFromInfluenceThreshold: requires 0 < "
        "influenceThreshold < sqrt(chi2 quantile).");
  }
  // Equation [d/dx \rho(r) = influenceThreshold] solved for c:
  //   c² = r²(t + sqrt(t r)) / (r - t)
  const double t = influenceThreshold;
  return r * std::sqrt((t + std::sqrt(t * r)) / (r - t));
}

/* ************************************************************************* */
// TruncatedLeastSquares
/* ************************************************************************* */

TruncatedLeastSquares::TruncatedLeastSquares(double c,
                                             const ReweightScheme reweight)
    : TruncatedLeastSquares(c, GradScheme::STANDARD, reweight) {}

TruncatedLeastSquares::TruncatedLeastSquares(
    double c, TruncatedLeastSquares::GradScheme graduation,
    const ReweightScheme reweight)
    : Base(reweight), c_(c), csquared_(c * c), graduation_(graduation) {
  if (c_ <= 0) {
    throw runtime_error(
        "mEstimator TruncatedLeastSquares takes only positive double in "
        "constructor.");
  }
}

double TruncatedLeastSquares::Weight(double distance2, double c2) {
  if (distance2 <= c2) {
    return 1.0;
  }
  return 0.0;
}

double TruncatedLeastSquares::Loss(double distance2, double c2) {
  if (distance2 <= c2) {
    return 0.5 * distance2;
  }
  return 0.5 * c2;
}

double TruncatedLeastSquares::GraduatedWeight(double distance2, double c2,
                                              double mu,
                                              GradScheme graduation) {
  const double m = ClampMu(mu);
  if (graduation == TruncatedLeastSquares::GradScheme::STANDARD) {
    if (m <= 0.0) return 1.0;
    return Weight(distance2, GradShapeSquared(c2, m));
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_LINEAR) {
    // Normalized Yang-TLS, i.e. the historical theta = mu / (1 - mu)
    if (m <= 0.0) return 1.0;
    if (m >= 1.0) return Weight(distance2, c2);
    const double lowerbound = m * c2;
    const double upperbound = c2 / m;

    if (distance2 <= lowerbound) return 1.0;
    if (distance2 >= upperbound) return 0.0;
    return (std::sqrt(m * c2 / distance2) - m) / (1.0 - m);
  } else {  // TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR
    // Normalized Peng MS-GNC-TLS, using the same theta = mu / (1 - mu).
    if (m >= 1.0) return Weight(distance2, c2);
    if (distance2 <= c2) return 1.0;
    if (m <= 0.0) return std::sqrt(c2 / distance2);
    if (distance2 >= c2 / (m * m)) return 0.0;
    return (std::sqrt(c2 / distance2) - m) / (1.0 - m);
  }
}

double TruncatedLeastSquares::GraduatedLoss(double distance2, double c2,
                                            double mu, GradScheme graduation) {
  const double m = ClampMu(mu);
  if (graduation == TruncatedLeastSquares::GradScheme::STANDARD) {
    if (m <= 0.0) return 0.5 * distance2;
    return Loss(distance2, GradShapeSquared(c2, m));
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_LINEAR) {
    // Normalized Yang-TLS, see GraduatedWeight for the endpoint conventions.
    if (m <= 0.0) return 0.5 * distance2;
    if (m >= 1.0) return Loss(distance2, c2);
    const double lowerbound = m * c2;
    const double upperbound = c2 / m;

    if (distance2 <= lowerbound) return 0.5 * distance2;
    if (distance2 >= upperbound) return 0.5 * c2;
    return 0.5 * (2.0 * std::sqrt(m * c2 * distance2) - m * (c2 + distance2)) /
           (1.0 - m);
  } else {  // TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR
    throw std::runtime_error(
        "TLS with GradScheme::GNC_SUPERLINEAR has no loss form");
  }
}

double TruncatedLeastSquares::weight(double distance) const {
  return Weight(distance * distance, csquared_);
}

double TruncatedLeastSquares::loss(double distance) const {
  return Loss(distance * distance, csquared_);
}

double TruncatedLeastSquares::graduatedWeight(double distance,
                                              double mu) const {
  return GraduatedWeight(distance * distance, csquared_, mu, graduation_);
}

double TruncatedLeastSquares::graduatedLoss(double distance, double mu) const {
  return GraduatedLoss(distance * distance, csquared_, mu, graduation_);
}

static std::string TruncatedLeastSquaresGradSchemeName(
    TruncatedLeastSquares::GradScheme graduation) {
  if (graduation == TruncatedLeastSquares::GradScheme::STANDARD) {
    return "STANDARD";
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_LINEAR) {
    return "GNC_LINEAR";
  } else if (graduation == TruncatedLeastSquares::GradScheme::GNC_SUPERLINEAR) {
    return "GNC_SUPERLINEAR";
  } else {
    throw std::runtime_error("Invalid TruncatedLeastSquares::GradScheme");
  }
}

void TruncatedLeastSquares::print(const std::string& s = "") const {
  std::cout << s << ": TLS (" << c_ << ", "
            << TruncatedLeastSquaresGradSchemeName(graduation_) << ")"
            << std::endl;
}

bool TruncatedLeastSquares::equals(const Base& expected, double tol) const {
  const TruncatedLeastSquares* p =
      dynamic_cast<const TruncatedLeastSquares*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol && graduation_ == p->graduation_;
}

TruncatedLeastSquares::shared_ptr TruncatedLeastSquares::Create(
    double c, const ReweightScheme reweight) {
  return shared_ptr(new TruncatedLeastSquares(c, reweight));
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

double DCS::Weight(double distance, double c) {
  const double e2 = distance * distance;
  if (e2 > c) {
    const double w = 2.0 * c / (c + e2);
    return w * w;
  }
  return 1.0;
}

double DCS::Loss(double distance, double c) {
  // This is the simplified version of Eq 9 from (Agarwal13icra)
  // after you simplify and cancel terms.
  const double e2 = distance * distance;
  const double e4 = e2 * e2;
  const double c2 = c * c;
  return (c2 * e2 + c * e4) / ((e2 + c) * (e2 + c));
}

double DCS::weight(double distance) const { return Weight(distance, c_); }

double DCS::loss(double distance) const { return Loss(distance, c_); }

double DCS::graduatedWeight(double distance, double mu) const {
  const double m = ClampMu(mu);
  if (m <= 0.0) return 1.0;
  // NOTE: DCS' shape parameter already has units of squared error.
  return Weight(distance, GradShapeSquared(c_, m));
}

double DCS::graduatedLoss(double distance, double mu) const {
  const double m = ClampMu(mu);
  // NOTE: DCS uses the x^2 (not 0.5 x^2) least-squares convention.
  if (m <= 0.0) return distance * distance;
  return Loss(distance, GradShapeSquared(c_, m));
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
               std::optional<std::function<double(double, double)>> gradWeight,
               std::optional<std::function<double(double, double)>> gradLoss,
               const ReweightScheme reweight, std::string name)
    : Base(reweight),
      weight_(std::move(weight)),
      loss_(loss),
      gradWeight_(gradWeight),
      gradLoss_(gradLoss),
      name_(std::move(name)) {}

Custom::Custom(std::function<double(double)> weight,
               std::function<double(double)> loss,
               const ReweightScheme reweight, std::string name)
    : Custom(std::move(weight), std::move(loss), std::nullopt, std::nullopt,
             reweight, std::move(name)) {}

double Custom::weight(double distance) const { return weight_(distance); }

double Custom::loss(double distance) const { return loss_(distance); }

double Custom::graduatedWeight(double distance, double mu) const {
  if (gradWeight_) {
    return (*gradWeight_)(distance, mu);
  } else {
    throw std::logic_error("Custom loss provided no graduated form.");
  }
}

double Custom::graduatedLoss(double distance, double mu) const {
  if (gradLoss_) {
    return (*gradLoss_)(distance, mu);
  } else {
    throw std::logic_error("Custom loss provided no graduated form.");
  }
}

void Custom::print(const std::string &s = "") const {
  std::cout << s << ": Custom (" << name_ << ")" << std::endl;
}

bool Custom::equals(const Base &expected, double /*tol*/) const {
  const auto *p = dynamic_cast<const Custom *>(&expected);
  if (p == nullptr)
    return false;
  // The graduated functions are optional, so they match when both are unset or
  // both hold the same target.
  auto sameGraduated =
      [](const std::optional<std::function<double(double, double)>> &a,
         const std::optional<std::function<double(double, double)>> &b) {
        if (a.has_value() != b.has_value()) return false;
        return !a.has_value() || a->target<double(double, double)>() ==
                                     b->target<double(double, double)>();
      };
  return name_ == p->name_ &&
         weight_.target<double(double)>() ==
             p->weight_.target<double(double)>() &&
         loss_.target<double(double)>() == p->loss_.target<double(double)>() &&
         sameGraduated(gradLoss_, p->gradLoss_) &&
         sameGraduated(gradWeight_, p->gradWeight_) &&
         reweight_ == p->reweight_;
}

Custom::shared_ptr Custom::Create(
    std::function<double(double)> weight, std::function<double(double)> loss,
    std::optional<std::function<double(double, double)>> gradWeight,
    std::optional<std::function<double(double, double)>> gradLoss,
    const ReweightScheme reweight, const std::string &name) {
  return std::make_shared<Custom>(std::move(weight), std::move(loss),
                                  std::move(gradWeight), std::move(gradLoss),
                                  reweight, name);
}

Custom::shared_ptr Custom::Create(std::function<double(double)> weight,
                                  std::function<double(double)> loss,
                                  const ReweightScheme reweight,
                                  const std::string& name) {
  return Create(std::move(weight), std::move(loss), std::nullopt, std::nullopt,
                reweight, name);
}

} // namespace mEstimator
} // namespace noiseModel
} // gtsam
