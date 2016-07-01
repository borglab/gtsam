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

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/timing.h>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <limits>
#include <iostream>
#include <typeinfo>
#include <stdexcept>
#include <cmath>

using namespace std;

namespace gtsam {
namespace noiseModel {

/* ************************************************************************* */
// update A, b
// A' \define A_{S}-ar and b'\define b-ad
// Linear algebra: takes away projection on latest orthogonal
// Graph: make a new factor on the separator S
// __attribute__ ((noinline))  // uncomment to prevent inlining when profiling
template<class MATRIX>
void updateAb(MATRIX& Ab, int j, const Vector& a, const Vector& rd) {
  size_t n = Ab.cols()-1;
  Ab.middleCols(j+1,n-j) -= a * rd.segment(j+1, n-j).transpose();
}

/* ************************************************************************* */
// check *above the diagonal* for non-zero entries
boost::optional<Vector> checkIfDiagonal(const Matrix M) {
  size_t m = M.rows(), n = M.cols();
  // check all non-diagonal entries
  bool full = false;
  size_t i, j;
  for (i = 0; i < m; i++)
    if (!full)
      for (j = i + 1; j < n; j++)
        if (std::abs(M(i, j)) > 1e-9) {
          full = true;
          break;
        }
  if (full) {
    return boost::none;
  } else {
    Vector diagonal(n);
    for (j = 0; j < n; j++)
      diagonal(j) = M(j, j);
    return diagonal;
  }
}

/* ************************************************************************* */
Vector Base::sigmas() const {
  throw("Base::sigmas: sigmas() not implemented for this noise model");
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::SqrtInformation(const Matrix& R, bool smart) {
  size_t m = R.rows(), n = R.cols();
  if (m != n)
    throw invalid_argument("Gaussian::SqrtInformation: R not square");
  if (smart) {
    boost::optional<Vector> diagonal = checkIfDiagonal(R);
    if (diagonal)
      return Diagonal::Sigmas(diagonal->array().inverse(), true);
  }
  // NOTE(frank): only reaches here if !(smart && diagonal)
  return shared_ptr(new Gaussian(R.rows(), R));
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Information(const Matrix& information, bool smart) {
  size_t m = information.rows(), n = information.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Information: R not square");
  boost::optional<Vector> diagonal = boost::none;
  if (smart)
    diagonal = checkIfDiagonal(information);
  if (diagonal)
    return Diagonal::Precisions(*diagonal, true);
  else {
    Eigen::LLT<Matrix> llt(information);
    Matrix R = llt.matrixU();
    return shared_ptr(new Gaussian(n, R));
  }
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Covariance(const Matrix& covariance,
    bool smart) {
  size_t m = covariance.rows(), n = covariance.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Covariance: covariance not square");
  boost::optional<Vector> variances = boost::none;
  if (smart)
    variances = checkIfDiagonal(covariance);
  if (variances)
    return Diagonal::Variances(*variances, true);
  else {
    // NOTE: if cov = L'*L, then the square root information R can be found by
    // QR, as L.inverse() = Q*R, with Q some rotation matrix. However, R has
    // annoying sign flips with respect the simpler Information(inv(cov)),
    // hence we choose the simpler path here:
    return Information(covariance.inverse(), false);
  }
}

/* ************************************************************************* */
void Gaussian::print(const string& name) const {
  gtsam::print(thisR(), name + "Gaussian");
}

/* ************************************************************************* */
bool Gaussian::equals(const Base& expected, double tol) const {
  const Gaussian* p = dynamic_cast<const Gaussian*> (&expected);
  if (p == NULL) return false;
  if (typeid(*this) != typeid(*p)) return false;
  //if (!sqrt_information_) return true; // ALEX todo;
  return equal_with_abs_tol(R(), p->R(), sqrt(tol));
}

/* ************************************************************************* */
Vector Gaussian::sigmas() const {
  // TODO(frank): can this be done faster?
  return Vector((thisR().transpose() * thisR()).inverse().diagonal()).cwiseSqrt();
}

/* ************************************************************************* */
Vector Gaussian::whiten(const Vector& v) const {
  return thisR() * v;
}

/* ************************************************************************* */
Vector Gaussian::unwhiten(const Vector& v) const {
  return backSubstituteUpper(thisR(), v);
}

/* ************************************************************************* */
double Gaussian::Mahalanobis(const Vector& v) const {
  // Note: for Diagonal, which does ediv_, will be correct for constraints
  Vector w = whiten(v);
  return w.dot(w);
}

/* ************************************************************************* */
Matrix Gaussian::Whiten(const Matrix& H) const {
  return thisR() * H;
}

/* ************************************************************************* */
void Gaussian::WhitenInPlace(Matrix& H) const {
  H = thisR() * H;
}

/* ************************************************************************* */
void Gaussian::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H = thisR() * H;
}

/* ************************************************************************* */
// General QR, see also special version in Constrained
SharedDiagonal Gaussian::QR(Matrix& Ab) const {

  gttic(Gaussian_noise_model_QR);

  static const bool debug = false;

  // get size(A) and maxRank
  // TODO: really no rank problems ?
   size_t m = Ab.rows(), n = Ab.cols()-1;
   size_t maxRank = min(m,n);

  // pre-whiten everything (cheaply if possible)
  WhitenInPlace(Ab);

  if(debug) gtsam::print(Ab, "Whitened Ab: ");

  // Eigen QR - much faster than older householder approach
  inplace_QR(Ab);
  Ab.triangularView<Eigen::StrictlyLower>().setZero();

  // hand-coded householder implementation
  // TODO: necessary to isolate last column?
  // householder(Ab, maxRank);

  return noiseModel::Unit::Create(maxRank);
}

void Gaussian::WhitenSystem(vector<Matrix>& A, Vector& b) const {
  for(Matrix& Aj: A) { WhitenInPlace(Aj); }
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A, Vector& b) const {
  WhitenInPlace(A);
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const {
  WhitenInPlace(A1);
  WhitenInPlace(A2);
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const{
  WhitenInPlace(A1);
  WhitenInPlace(A2);
  WhitenInPlace(A3);
  whitenInPlace(b);
}

/* ************************************************************************* */
// Diagonal
/* ************************************************************************* */
Diagonal::Diagonal() :
    Gaussian(1) // TODO: Frank asks: really sure about this?
{
}

/* ************************************************************************* */
Diagonal::Diagonal(const Vector& sigmas)
    : Gaussian(sigmas.size()),
      sigmas_(sigmas),
      invsigmas_(sigmas.array().inverse()),
      precisions_(invsigmas_.array().square()) {
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Variances(const Vector& variances, bool smart) {
  if (smart) {
    // check whether all the same entry
    size_t n = variances.size();
    for (size_t j = 1; j < n; j++)
      if (variances(j) != variances(0)) goto full;
    return Isotropic::Variance(n, variances(0), true);
  }
  full: return shared_ptr(new Diagonal(variances.cwiseSqrt()));
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Sigmas(const Vector& sigmas, bool smart) {
  if (smart) {
    size_t n = sigmas.size();
    if (n==0) goto full;
    // look for zeros to make a constraint
    for (size_t j=0; j< n; ++j)
      if (sigmas(j)<1e-8)
        return Constrained::MixedSigmas(sigmas);
    // check whether all the same entry
    for (size_t j = 1; j < n; j++)
      if (sigmas(j) != sigmas(0)) goto full;
    return Isotropic::Sigma(n, sigmas(0), true);
  }
  full: return Diagonal::shared_ptr(new Diagonal(sigmas));
}

/* ************************************************************************* */
void Diagonal::print(const string& name) const {
  gtsam::print(sigmas_, name + "diagonal sigmas");
}

/* ************************************************************************* */
Vector Diagonal::whiten(const Vector& v) const {
  return v.cwiseProduct(invsigmas_);
}

/* ************************************************************************* */
Vector Diagonal::unwhiten(const Vector& v) const {
  return v.cwiseProduct(sigmas_);
}

/* ************************************************************************* */
Matrix Diagonal::Whiten(const Matrix& H) const {
  return vector_scale(invsigmas(), H);
}

/* ************************************************************************* */
void Diagonal::WhitenInPlace(Matrix& H) const {
  vector_scale_inplace(invsigmas(), H);
}

/* ************************************************************************* */
void Diagonal::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H = invsigmas().asDiagonal() * H;
}

/* ************************************************************************* */
// Constrained
/* ************************************************************************* */

namespace internal {
// switch precisions and invsigmas to finite value
// TODO: why?? And, why not just ask s==0.0 below ?
static void fix(const Vector& sigmas, Vector& precisions, Vector& invsigmas) {
  for (Vector::Index i = 0; i < sigmas.size(); ++i)
    if (!std::isfinite(1. / sigmas[i])) {
      precisions[i] = 0.0;
      invsigmas[i] = 0.0;
    }
}
}

/* ************************************************************************* */
Constrained::Constrained(const Vector& sigmas)
  : Diagonal(sigmas), mu_(Vector::Constant(sigmas.size(), 1000.0)) {
  internal::fix(sigmas, precisions_, invsigmas_);
}

/* ************************************************************************* */
Constrained::Constrained(const Vector& mu, const Vector& sigmas)
  : Diagonal(sigmas), mu_(mu) {
  internal::fix(sigmas, precisions_, invsigmas_);
}

/* ************************************************************************* */
Constrained::shared_ptr Constrained::MixedSigmas(const Vector& mu,
    const Vector& sigmas) {
  return shared_ptr(new Constrained(mu, sigmas));
}

/* ************************************************************************* */
bool Constrained::constrained(size_t i) const {
  // TODO why not just check sigmas_[i]==0.0 ?
  return !std::isfinite(1./sigmas_[i]);
}

/* ************************************************************************* */
void Constrained::print(const std::string& name) const {
  gtsam::print(sigmas_, name + "constrained sigmas");
  gtsam::print(mu_, name + "constrained mu");
}

/* ************************************************************************* */
Vector Constrained::whiten(const Vector& v) const {
  // If sigmas[i] is not 0 then divide v[i] by sigmas[i], as usually done in
  // other normal Gaussian noise model. Otherwise, sigmas[i] = 0 indicating
  // a hard constraint, we don't do anything.
  const Vector& a = v;
  const Vector& b = sigmas_;
  size_t n = a.size();
  assert (b.size()==a.size());
  Vector c(n);
  for( size_t i = 0; i < n; i++ ) {
    const double& ai = a(i), bi = b(i);
    c(i) = (bi==0.0) ? ai : ai/bi; // NOTE: not ediv_()
  }
  return c;
}

/* ************************************************************************* */
double Constrained::distance(const Vector& v) const {
  Vector w = Diagonal::whiten(v); // get noisemodel for constrained elements
  for (size_t i=0; i<dim_; ++i)  // add mu weights on constrained variables
    if (constrained(i)) // whiten makes constrained variables zero
      w[i] = v[i] * sqrt(mu_[i]); // TODO: may want to store sqrt rather than rebuild
  return w.dot(w);
}

/* ************************************************************************* */
Matrix Constrained::Whiten(const Matrix& H) const {
  Matrix A = H;
  for (DenseIndex i=0; i<(DenseIndex)dim_; ++i)
    if (!constrained(i)) // if constrained, leave row of A as is
      A.row(i) *= invsigmas_(i);
  return A;
}

/* ************************************************************************* */
void Constrained::WhitenInPlace(Matrix& H) const {
  for (DenseIndex i=0; i<(DenseIndex)dim_; ++i)
    if (!constrained(i)) // if constrained, leave row of H as is
      H.row(i) *= invsigmas_(i);
}

/* ************************************************************************* */
void Constrained::WhitenInPlace(Eigen::Block<Matrix> H) const {
  for (DenseIndex i=0; i<(DenseIndex)dim_; ++i)
    if (!constrained(i)) // if constrained, leave row of H as is
      H.row(i) *= invsigmas_(i);
}

/* ************************************************************************* */
Constrained::shared_ptr Constrained::unit() const {
  Vector sigmas = Vector::Ones(dim());
  for (size_t i=0; i<dim(); ++i)
    if (constrained(i))
      sigmas(i) = 0.0;
  return MixedSigmas(mu_, sigmas);
}

/* ************************************************************************* */
// Special version of QR for Constrained calls slower but smarter code
// that deals with possibly zero sigmas
// It is Gram-Schmidt orthogonalization rather than Householder

// Check whether column a triggers a constraint and corresponding variable is deterministic
// Return constraint_row with maximum element in case variable plays in multiple constraints
template <typename VECTOR>
boost::optional<size_t> check_if_constraint(VECTOR a, const Vector& invsigmas, size_t m) {
  boost::optional<size_t> constraint_row;
  // not zero, so roundoff errors will not be counted
  // TODO(frank): that's a fairly crude way of dealing with roundoff errors :-(
  double max_element = 1e-9;
  for (size_t i = 0; i < m; i++) {
    if (!std::isinf(invsigmas[i]))
      continue;
    double abs_ai = std::abs(a(i,0));
    if (abs_ai > max_element) {
      max_element = abs_ai;
      constraint_row.reset(i);
    }
  }
  return constraint_row;
}

SharedDiagonal Constrained::QR(Matrix& Ab) const {
  static const double kInfinity = std::numeric_limits<double>::infinity();

  // get size(A) and maxRank
  size_t m = Ab.rows();
  const size_t n = Ab.cols() - 1;
  const size_t maxRank = min(m, n);

  // create storage for [R d]
  typedef boost::tuple<size_t, Matrix, double> Triple;
  list<Triple> Rd;

  Matrix rd(1, n + 1);  // and for row of R
  Vector invsigmas = sigmas_.array().inverse();
  Vector weights = invsigmas.array().square();  // calculate weights once

  // We loop over all columns, because the columns that can be eliminated
  // are not necessarily contiguous. For each one, estimate the corresponding
  // scalar variable x as d-rS, with S the separator (remaining columns).
  // Then update A and b by substituting x with d-rS, zero-ing out x's column.
  for (size_t j = 0; j < n; ++j) {
    // extract the first column of A
    Eigen::Block<Matrix> a = Ab.block(0, j, m, 1);

    // Check whether we need to handle as a constraint
    boost::optional<size_t> constraint_row = check_if_constraint(a, invsigmas, m);

    if (constraint_row) {
      // Handle this as a constraint, as the i^th row has zero sigma with non-zero entry A(i,j)

      // In this case, the row in [R|d] is simply the row in [A|b]
      // NOTE(frank): we used to divide by a[i] but there is no need with a constraint
      rd = Ab.row(*constraint_row);

      // Construct solution (r, d, sigma)
      Rd.push_back(boost::make_tuple(j, rd, kInfinity));

      // exit after rank exhausted
      if (Rd.size() >= maxRank)
        break;

      // The constraint row will be zeroed out, so we can save work by swapping in the
      // last valid row and decreasing m. This will save work on subsequent down-dates, too.
      m -= 1;
      if (*constraint_row != m) {
        Ab.row(*constraint_row) = Ab.row(m);
        weights(*constraint_row) = weights(m);
        invsigmas(*constraint_row) = invsigmas(m);
      }

      // get a reduced a-column which is now shorter
      Eigen::Block<Matrix> a_reduced = Ab.block(0, j, m, 1);
      a_reduced *= (1.0/rd(0, j)); // NOTE(frank): this is the 1/a[i] = 1/rd(0,j) factor we need!

      // Rank-1 down-date of Ab, expensive, using outer product
      Ab.block(0, j + 1, m, n - j).noalias() -= a_reduced * rd.middleCols(j + 1, n - j);
    } else {
      // Treat in normal Gram-Schmidt way
      // Calculate weighted pseudo-inverse and corresponding precision

      // Form psuedo-inverse inv(a'inv(Sigma)a)a'inv(Sigma)
      // For diagonal Sigma, inv(Sigma) = diag(precisions)
      double precision = 0;
      Vector pseudo(m);     // allocate storage for pseudo-inverse
      for (size_t i = 0; i < m; i++) {
        double ai = a(i, 0);
        if (std::abs(ai) > 1e-9) {  // also catches remaining sigma==0 rows
          pseudo[i] = weights[i] * ai;
          precision += pseudo[i] * ai;
        } else
          pseudo[i] = 0;
      }

      if (precision > 1e-8) {
        pseudo /= precision;

        // create solution [r d], rhs is automatically r(n)
        rd(0, j) = 1.0;  // put 1 on diagonal
        rd.block(0, j + 1, 1, n - j) = pseudo.transpose() * Ab.block(0, j + 1, m, n - j);

        // construct solution (r, d, sigma)
        Rd.push_back(boost::make_tuple(j, rd, precision));
      } else {
        // If precision is zero, no information on this column
        // This is actually not limited to constraints, could happen in Gaussian::QR
        // In that case, we're probably hosed. TODO: make sure Householder is rank-revealing
        continue;  // but even if not, no need to update if a==zeros
      }

      // exit after rank exhausted
      if (Rd.size() >= maxRank)
        break;

      // Rank-1 down-date of Ab, expensive, using outer product
      Ab.block(0, j + 1, m, n - j).noalias() -= a * rd.middleCols(j + 1, n - j);
    }
  }

  // Create storage for precisions
  Vector precisions(Rd.size());

  // Write back result in Ab, imperative as we are
  size_t i = 0;  // start with first row
  bool mixed = false;
  Ab.setZero();  // make sure we don't look below
  for (const Triple& t: Rd) {
    const size_t& j = t.get<0>();
    const Matrix& rd = t.get<1>();
    precisions(i) = t.get<2>();
    if (std::isinf(precisions(i)))
      mixed = true;
    Ab.block(i, j, 1, n + 1 - j) = rd.block(0, j, 1, n + 1 - j);
    i += 1;
  }

  // Must include mu, as the defaults might be higher, resulting in non-convergence
  return mixed ? Constrained::MixedPrecisions(mu_, precisions) : Diagonal::Precisions(precisions);
}

/* ************************************************************************* */
// Isotropic
/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Sigma(size_t dim, double sigma, bool smart)  {
  if (smart && std::abs(sigma-1.0)<1e-9) return Unit::Create(dim);
  return shared_ptr(new Isotropic(dim, sigma));
}

/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Variance(size_t dim, double variance, bool smart)  {
  if (smart && std::abs(variance-1.0)<1e-9) return Unit::Create(dim);
  return shared_ptr(new Isotropic(dim, sqrt(variance)));
}

/* ************************************************************************* */
void Isotropic::print(const string& name) const {
  cout << boost::format("isotropic dim=%1% sigma=%2%") % dim() % sigma_ << endl;
}

/* ************************************************************************* */
double Isotropic::Mahalanobis(const Vector& v) const {
  return v.dot(v) * invsigma_ * invsigma_;
}

/* ************************************************************************* */
Vector Isotropic::whiten(const Vector& v) const {
  return v * invsigma_;
}

/* ************************************************************************* */
Vector Isotropic::unwhiten(const Vector& v) const {
  return v * sigma_;
}

/* ************************************************************************* */
Matrix Isotropic::Whiten(const Matrix& H) const {
  return invsigma_ * H;
}

/* ************************************************************************* */
void Isotropic::WhitenInPlace(Matrix& H) const {
  H *= invsigma_;
}

/* ************************************************************************* */
void Isotropic::whitenInPlace(Vector& v) const {
  v *= invsigma_;
}

/* ************************************************************************* */
void Isotropic::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H *= invsigma_;
}

/* ************************************************************************* */
// Unit
/* ************************************************************************* */
void Unit::print(const std::string& name) const {
  cout << name << "unit (" << dim_ << ") " << endl;
}

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

double GemanMcClure::weight(double error) const {
  const double c2 = c_*c_;
  const double c4 = c2*c2;
  const double c2error = c2 + error*error;
  return c4/(c2error*c2error);
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

double DCS::weight(double error) const {
  const double e2 = error*error;
  if (e2 > c_)
  {
    const double w = 2.0*c_/(c_ + e2);
    return w*w;
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

/* ************************************************************************* */
// Robust
/* ************************************************************************* */

void Robust::print(const std::string& name) const {
  robust_->print(name);
  noise_->print(name);
}

bool Robust::equals(const Base& expected, double tol) const {
  const Robust* p = dynamic_cast<const Robust*> (&expected);
  if (p == NULL) return false;
  return noise_->equals(*p->noise_,tol) && robust_->equals(*p->robust_,tol);
}

void Robust::WhitenSystem(Vector& b) const {
  noise_->whitenInPlace(b);
  robust_->reweight(b);
}

void Robust::WhitenSystem(vector<Matrix>& A, Vector& b) const {
  noise_->WhitenSystem(A,b);
  robust_->reweight(A,b);
}

void Robust::WhitenSystem(Matrix& A, Vector& b) const {
  noise_->WhitenSystem(A,b);
  robust_->reweight(A,b);
}

void Robust::WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const {
  noise_->WhitenSystem(A1,A2,b);
  robust_->reweight(A1,A2,b);
}

void Robust::WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const{
  noise_->WhitenSystem(A1,A2,A3,b);
  robust_->reweight(A1,A2,A3,b);
}

Robust::shared_ptr Robust::Create(
  const RobustModel::shared_ptr &robust, const NoiseModel::shared_ptr noise){
  return shared_ptr(new Robust(robust,noise));
}

/* ************************************************************************* */

}
} // gtsam
