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

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <typeinfo>

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
std::optional<Vector> checkIfDiagonal(const Matrix& M) {
  size_t m = M.rows(), n = M.cols();
  assert(m > 0);
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
    return {};
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
double Base::squaredMahalanobisDistance(const Vector& v) const {
  // Note: for Diagonal, which does ediv_, will be correct for constraints
  Vector w = whiten(v);
  return w.dot(w);
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::SqrtInformation(const Matrix& R, bool smart) {
  size_t m = R.rows(), n = R.cols();
  if (m != n)
    throw invalid_argument("Gaussian::SqrtInformation: R not square");
  if (smart) {
    std::optional<Vector> diagonal = checkIfDiagonal(R);
    if (diagonal)
      return Diagonal::Sigmas(diagonal->array().inverse(), true);
  }
  // NOTE(frank): only reaches here if !(smart && diagonal)
  return std::make_shared<Gaussian>(R.rows(), R);
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Information(const Matrix& information, bool smart) {
  size_t m = information.rows(), n = information.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Information: R not square");
  std::optional<Vector> diagonal = {};
  if (smart)
    diagonal = checkIfDiagonal(information);
  if (diagonal)
    return Diagonal::Precisions(*diagonal, true);
  else {
    Eigen::LLT<Matrix> llt(information);
    Matrix R = llt.matrixU();
    return std::make_shared<Gaussian>(n, R);
  }
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Covariance(const Matrix& covariance,
    bool smart) {
  size_t m = covariance.rows(), n = covariance.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Covariance: covariance not square");
  std::optional<Vector> variances = {};
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
  gtsam::print(thisR(), name + "Gaussian ");
}

/* ************************************************************************* */
bool Gaussian::equals(const Base& expected, double tol) const {
  const Gaussian* p = dynamic_cast<const Gaussian*> (&expected);
  if (p == nullptr) return false;
  if (typeid(*this) != typeid(*p)) return false;
  return equal_with_abs_tol(R(), p->R(), sqrt(tol));
}

/* ************************************************************************* */
Matrix Gaussian::covariance() const {
  // Uses a fast version of `covariance = information().inverse();`
  const Matrix& R = this->R();
  Matrix I = Matrix::Identity(R.rows(), R.cols());
  // Fast inverse of upper-triangular matrix R using forward-substitution
  Matrix Rinv = R.triangularView<Eigen::Upper>().solve(I);
  // (R' * R)^{-1} = R^{-1} * R^{-1}'
  return Rinv * Rinv.transpose();
}

/* ************************************************************************* */
Vector Gaussian::sigmas() const {
  return Vector(covariance().diagonal()).cwiseSqrt();
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

Matrix Gaussian::information() const { return R().transpose() * R(); }

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
Diagonal::shared_ptr Diagonal::Precisions(const Vector& precisions,
                                          bool smart) {
  return Variances(precisions.array().inverse(), smart);
}
/* ************************************************************************* */
void Diagonal::print(const string& name) const {
  gtsam::print(sigmas_, name + "diagonal sigmas ");
}

/* ************************************************************************* */
Vector Diagonal::whiten(const Vector& v) const {
  return v.cwiseProduct(invsigmas_);
}

Vector Diagonal::unwhiten(const Vector& v) const {
  return v.cwiseProduct(sigmas_);
}

Matrix Diagonal::Whiten(const Matrix& H) const {
  return vector_scale(invsigmas(), H);
}

void Diagonal::WhitenInPlace(Matrix& H) const {
  vector_scale_inplace(invsigmas(), H);
}

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
  gtsam::print(sigmas_, name + "constrained sigmas ");
  gtsam::print(mu_, name + "constrained mu ");
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
Constrained::shared_ptr Constrained::MixedSigmas(const Vector& sigmas) {
  return MixedSigmas(Vector::Constant(sigmas.size(), 1000.0), sigmas);
}

Constrained::shared_ptr Constrained::MixedSigmas(double m,
                                                 const Vector& sigmas) {
  return MixedSigmas(Vector::Constant(sigmas.size(), m), sigmas);
}

Constrained::shared_ptr Constrained::MixedVariances(const Vector& mu,
                                                    const Vector& variances) {
  return shared_ptr(new Constrained(mu, variances.cwiseSqrt()));
}
Constrained::shared_ptr Constrained::MixedVariances(const Vector& variances) {
  return shared_ptr(new Constrained(variances.cwiseSqrt()));
}

Constrained::shared_ptr Constrained::MixedPrecisions(const Vector& mu,
                                                     const Vector& precisions) {
  return MixedVariances(mu, precisions.array().inverse());
}
Constrained::shared_ptr Constrained::MixedPrecisions(const Vector& precisions) {
  return MixedVariances(precisions.array().inverse());
}

/* ************************************************************************* */
double Constrained::squaredMahalanobisDistance(const Vector& v) const {
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
std::optional<size_t> check_if_constraint(VECTOR a, const Vector& invsigmas, size_t m) {
  std::optional<size_t> constraint_row;
  // not zero, so roundoff errors will not be counted
  // TODO(frank): that's a fairly crude way of dealing with roundoff errors :-(
  double max_element = 1e-9;
  for (size_t i = 0; i < m; i++) {
    if (!std::isinf(invsigmas[i]))
      continue;
    double abs_ai = std::abs(a(i,0));
    if (abs_ai > max_element) {
      max_element = abs_ai;
      constraint_row = i;
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
  typedef std::tuple<size_t, Matrix, double> Triple;
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
    std::optional<size_t> constraint_row = check_if_constraint(a, invsigmas, m);

    if (constraint_row) {
      // Handle this as a constraint, as the i^th row has zero sigma with non-zero entry A(i,j)

      // In this case, the row in [R|d] is simply the row in [A|b]
      // NOTE(frank): we used to divide by a[i] but there is no need with a constraint
      rd = Ab.row(*constraint_row);

      // Construct solution (r, d, sigma)
      Rd.push_back(std::make_tuple(j, rd, kInfinity));

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
        Rd.push_back(std::make_tuple(j, rd, precision));
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
    const size_t& j = std::get<0>(t);
    const Matrix& rd = std::get<1>(t);
    precisions(i) = std::get<2>(t);
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
double Isotropic::squaredMahalanobisDistance(const Vector& v) const {
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
double Unit::squaredMahalanobisDistance(const Vector& v) const {
  return v.dot(v);
}

/* ************************************************************************* */
// Robust
/* ************************************************************************* */

void Robust::print(const std::string& name) const {
  robust_->print(name);
  noise_->print(name);
}

bool Robust::equals(const Base& expected, double tol) const {
  const Robust* p = dynamic_cast<const Robust*> (&expected);
  if (p == nullptr) return false;
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

Vector Robust::unweightedWhiten(const Vector& v) const {
  return noise_->unweightedWhiten(v);
}
double Robust::weight(const Vector& v) const {
  return robust_->weight(v.norm());
}

Robust::shared_ptr Robust::Create(
const RobustModel::shared_ptr &robust, const NoiseModel::shared_ptr noise){
  return shared_ptr(new Robust(robust,noise));
}

/* ************************************************************************* */

}
} // gtsam
