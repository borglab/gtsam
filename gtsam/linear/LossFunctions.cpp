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

#include <gtsam/linear/LossFunctions.h>

#include <iostream>
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

double Huber::weight(double distance) const {
  const double absError = std::abs(distance);
  return (absError <= k_) ? (1.0) : (k_ / absError);
}

double Huber::loss(double distance) const {
  const double absError = std::abs(distance);
  if (absError <= k_) {  // |x| <= k
    return distance*distance / 2;
  } else { // |x| > k
    return k_ * (absError - (k_/2));
  }
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

double Cauchy::weight(double distance) const {
  return ksquared_ / (ksquared_ + distance*distance);
}

double Cauchy::loss(double distance) const {
  const double val = std::log1p(distance * distance / ksquared_);
  return ksquared_ * val * 0.5;
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

double Tukey::weight(double distance) const {
  if (std::abs(distance) <= c_) {
    const double one_minus_xc2 = 1.0 - distance*distance/csquared_;
    return one_minus_xc2 * one_minus_xc2;
  }
  return 0.0;
}

double Tukey::loss(double distance) const {
  double absError = std::abs(distance);
  if (absError <= c_) {
    const double one_minus_xc2 = 1.0 - distance*distance/csquared_;
    const double t = one_minus_xc2*one_minus_xc2*one_minus_xc2;
    return csquared_ * (1 - t) / 6.0;
  } else {
    return csquared_ / 6.0;
  }
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

double Welsch::weight(double distance) const {
  const double xc2 = (distance*distance)/csquared_;
  return std::exp(-xc2);
}

double Welsch::loss(double distance) const {
  const double xc2 = (distance*distance)/csquared_;
  return csquared_ * 0.5 * -std::expm1(-xc2);
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
  : Base(reweight), c_(c) {
}

double GemanMcClure::weight(double distance) const {
  const double c2 = c_*c_;
  const double c4 = c2*c2;
  const double c2error = c2 + distance*distance;
  return c4/(c2error*c2error);
}

double GemanMcClure::loss(double distance) const {
  const double c2 = c_*c_;
  const double error2 = distance*distance;
  return 0.5 * (c2 * error2) / (c2 + error2);
}

void GemanMcClure::print(const std::string &s="") const {
  std::cout << s << ": Geman-McClure (" << c_ << ")" << std::endl;
}

bool GemanMcClure::equals(const Base &expected, double tol) const {
  const GemanMcClure* p = dynamic_cast<const GemanMcClure*>(&expected);
  if (p == nullptr) return false;
  return std::abs(c_ - p->c_) < tol;
}

GemanMcClure::shared_ptr GemanMcClure::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new GemanMcClure(c, reweight));
}

/* ************************************************************************* */
// DCS
/* ************************************************************************* */
DCS::DCS(double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
}

double DCS::weight(double distance) const {
  const double e2 = distance*distance;
  if (e2 > c_)
  {
    const double w = 2.0*c_/(c_ + e2);
    return w*w;
  }

  return 1.0;
}

double DCS::loss(double distance) const {
  // This is the simplified version of Eq 9 from (Agarwal13icra)
  // after you simplify and cancel terms.
  const double e2 = distance*distance;
  const double e4 = e2*e2;
  const double c2 = c_*c_;

  return (c2*e2 + c_*e4) / ((e2 + c_)*(e2 + c_));
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

Custom::Custom(std::function<double(double)> weight, std::function<double(double)> loss, const ReweightScheme reweight,
               std::string name)
    : Base(reweight), weight_(std::move(weight)), loss_(loss), name_(std::move(name)) {}

double Custom::weight(double distance) const {
  return weight_(distance);
}

double Custom::loss(double distance) const {
  return loss_(distance);
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

Custom::shared_ptr Custom::Create(std::function<double(double)> weight, std::function<double(double)> loss,
                                  const ReweightScheme reweight, const std::string &name) {
  return std::make_shared<Custom>(std::move(weight), std::move(loss), reweight, name);
}

} // namespace mEstimator
} // namespace noiseModel
} // gtsam
