#include <gtsam/linear/MEstimators.h>

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

Fair::Fair(double c, const ReweightScheme reweight) : Base(reweight), c_(c) {
  if (c_ <= 0) {
    throw runtime_error("mEstimator Fair takes only positive double in constructor.");
  }
}

/* ************************************************************************* */
// Fair
/* ************************************************************************* */

void Fair::print(const std::string &s="") const
{ cout << s << "fair (" << c_ << ")" << endl; }

bool Fair::equals(const Base &expected, double tol) const {
  const Fair* p = dynamic_cast<const Fair*> (&expected);
  if (p == NULL) return false;
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

void Huber::print(const std::string &s="") const {
  cout << s << "huber (" << k_ << ")" << endl;
}

bool Huber::equals(const Base &expected, double tol) const {
  const Huber* p = dynamic_cast<const Huber*>(&expected);
  if (p == NULL) return false;
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

void Cauchy::print(const std::string &s="") const {
  cout << s << "cauchy (" << k_ << ")" << endl;
}

bool Cauchy::equals(const Base &expected, double tol) const {
  const Cauchy* p = dynamic_cast<const Cauchy*>(&expected);
  if (p == NULL) return false;
  return std::abs(ksquared_ - p->ksquared_) < tol;
}

Cauchy::shared_ptr Cauchy::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Cauchy(c, reweight));
}

/* ************************************************************************* */
// Tukey
/* ************************************************************************* */
Tukey::Tukey(double c, const ReweightScheme reweight) : Base(reweight), c_(c), csquared_(c * c) {}

void Tukey::print(const std::string &s="") const {
  std::cout << s << ": Tukey (" << c_ << ")" << std::endl;
}

bool Tukey::equals(const Base &expected, double tol) const {
  const Tukey* p = dynamic_cast<const Tukey*>(&expected);
  if (p == NULL) return false;
  return std::abs(c_ - p->c_) < tol;
}

Tukey::shared_ptr Tukey::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Tukey(c, reweight));
}

/* ************************************************************************* */
// Welsh
/* ************************************************************************* */
Welsh::Welsh(double c, const ReweightScheme reweight) : Base(reweight), c_(c), csquared_(c * c) {}

void Welsh::print(const std::string &s="") const {
  std::cout << s << ": Welsh (" << c_ << ")" << std::endl;
}

bool Welsh::equals(const Base &expected, double tol) const {
  const Welsh* p = dynamic_cast<const Welsh*>(&expected);
  if (p == NULL) return false;
  return std::abs(c_ - p->c_) < tol;
}

Welsh::shared_ptr Welsh::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new Welsh(c, reweight));
}

/* ************************************************************************* */
// GemanMcClure
/* ************************************************************************* */
GemanMcClure::GemanMcClure(double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
}

double GemanMcClure::sqrtWeight(double error) const {
  const double c2 = c_*c_;
  const double c2error = c2 + error*error;
  return c2/c2error;
}

void GemanMcClure::print(const std::string &s="") const {
  std::cout << s << ": Geman-McClure (" << c_ << ")" << std::endl;
}

bool GemanMcClure::equals(const Base &expected, double tol) const {
  const GemanMcClure* p = dynamic_cast<const GemanMcClure*>(&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_) < tol;
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

double DCS::sqrtWeight(double error) const {
  const double e2 = error*error;
  if (e2 > c_)
  {
    const double w = 2.0*c_/(c_ + e2);
    return w;
  }

  return 1.0;
}

void DCS::print(const std::string &s="") const {
  std::cout << s << ": DCS (" << c_ << ")" << std::endl;
}

bool DCS::equals(const Base &expected, double tol) const {
  const DCS* p = dynamic_cast<const DCS*>(&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_) < tol;
}

DCS::shared_ptr DCS::Create(double c, const ReweightScheme reweight) {
  return shared_ptr(new DCS(c, reweight));
}

/* ************************************************************************* */
// L2WithDeadZone
/* ************************************************************************* */

L2WithDeadZone::L2WithDeadZone(double k, const ReweightScheme reweight) : Base(reweight), k_(k) {
  if (k_ <= 0) {
    throw runtime_error("mEstimator L2WithDeadZone takes only positive double in constructor.");
  }
}

void L2WithDeadZone::print(const std::string &s="") const {
  std::cout << s << ": L2WithDeadZone (" << k_ << ")" << std::endl;
}

bool L2WithDeadZone::equals(const Base &expected, double tol) const {
  const L2WithDeadZone* p = dynamic_cast<const L2WithDeadZone*>(&expected);
  if (p == NULL) return false;
  return fabs(k_ - p->k_) < tol;
}

L2WithDeadZone::shared_ptr L2WithDeadZone::Create(double k, const ReweightScheme reweight) {
  return shared_ptr(new L2WithDeadZone(k, reweight));
}

} // namespace mEstimator

} // noiseModel
} // gtsam
