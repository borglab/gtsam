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

#include <limits>
#include <iostream>
#include <typeinfo>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <gtsam/base/timing.h>
#include <gtsam/linear/NoiseModel.h>

static double inf = std::numeric_limits<double>::infinity();
using namespace std;

namespace gtsam {
namespace noiseModel {

/* ************************************************************************* */
// update A, b
// A' \define A_{S}-ar and b'\define b-ad
// Linear algebra: takes away projection on latest orthogonal
// Graph: make a new factor on the separator S
// __attribute__ ((noinline))	// uncomment to prevent inlining when profiling
template<class MATRIX>
void updateAb(MATRIX& Ab, int j, const Vector& a, const Vector& rd) {
	size_t n = Ab.cols()-1;
	Ab.middleCols(j+1,n-j) -= a * rd.segment(j+1, n-j).transpose();
}


/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Covariance(const Matrix& covariance, bool smart) {
	size_t m = covariance.rows(), n = covariance.cols();
	if (m != n) throw invalid_argument("Gaussian::Covariance: covariance not square");
	if (smart) {
		// check all non-diagonal entries
		size_t i,j;
		for (i = 0; i < m; i++)
			for (j = 0; j < n; j++)
				if (i != j && fabs(covariance(i, j)) > 1e-9) goto full;
		Vector variances(n);
		for (j = 0; j < n; j++) variances(j) = covariance(j,j);
		return Diagonal::Variances(variances,true);
	}
	full: return shared_ptr(new Gaussian(n, inverse_square_root(covariance)));
}

/* ************************************************************************* */
void Gaussian::print(const string& name) const {
	gtsam::print(thisR(), "Gaussian");
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
// General QR, see also special version in Constrained
SharedDiagonal Gaussian::QR(Matrix& Ab) const {

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

	// hand-coded householder implementation
	// TODO: necessary to isolate last column?
//	householder(Ab, maxRank);

	return Unit::Create(maxRank);
}

void Gaussian::WhitenSystem(vector<Matrix>& A, Vector& b) const {
  BOOST_FOREACH(Matrix& Aj, A) { WhitenInPlace(Aj); }
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
		Gaussian(1), sigmas_(ones(1)), invsigmas_(ones(1)) {
}

Diagonal::Diagonal(const Vector& sigmas, bool initialize_invsigmas):
	Gaussian(sigmas.size()), sigmas_(sigmas) {
	if (initialize_invsigmas)
		invsigmas_ = reciprocal(sigmas);
	else
		invsigmas_ = boost::none;
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Variances(const Vector& variances, bool smart) {
	if (smart) {
		// check whether all the same entry
		int j, n = variances.size();
		for (j = 1; j < n; j++)
			if (variances(j) != variances(0)) goto full;
		return Isotropic::Variance(n, variances(0), true);
	}
	full: return shared_ptr(new Diagonal(esqrt(variances)));
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Sigmas(const Vector& sigmas, bool smart) {
	if (smart) {
		// look for zeros to make a constraint
		for (size_t i=0; i< (size_t) sigmas.size(); ++i)
			if (sigmas(i)<1e-8)
				return Constrained::MixedSigmas(sigmas);
	}
	return Diagonal::shared_ptr(new Diagonal(sigmas));
}

/* ************************************************************************* */
void Diagonal::print(const string& name) const {
	gtsam::print(sigmas_, name + "diagonal sigmas");
}

/* ************************************************************************* */
Vector Diagonal::invsigmas() const {
	if (invsigmas_) return *invsigmas_;
	else return reciprocal(sigmas_);
}

/* ************************************************************************* */
double Diagonal::invsigma(size_t i) const {
	if (invsigmas_) return (*invsigmas_)(i);
	else return 1.0/sigmas_(i);
}

/* ************************************************************************* */
Vector Diagonal::whiten(const Vector& v) const {
	return emul(v, invsigmas());
}

/* ************************************************************************* */
Vector Diagonal::unwhiten(const Vector& v) const {
	return emul(v, sigmas_);
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
// Constrained
/* ************************************************************************* */
Constrained::shared_ptr Constrained::MixedSigmas(const Vector& mu, const Vector& sigmas, bool smart) {
	// FIXME: can't return a diagonal shared_ptr due to conversion
//	if (smart) {
//		// look for zeros to make a constraint
//		for (size_t i=0; i< (size_t) sigmas.size(); ++i)
//			if (sigmas(i)<1e-8)
//				return MixedSigmas(mu, sigmas);
//		return Diagonal::Sigmas(sigmas);
//	}
	return shared_ptr(new Constrained(mu, sigmas));
}

/* ************************************************************************* */
void Constrained::print(const std::string& name) const {
	gtsam::print(sigmas_, name + "constrained sigmas");
	gtsam::print(mu_, name + "constrained mu");
}

/* ************************************************************************* */
Vector Constrained::whiten(const Vector& v) const {
	const Vector& a = v;
	const Vector& b = sigmas_;
	size_t n = a.size();
	assert (b.size()==a.size());
	Vector c(n);
	for( size_t i = 0; i < n; i++ ) {
		const double& ai = a(i), &bi = b(i);
		c(i) = (bi==0.0) ? ai : ai/bi; // NOTE: not ediv_()
	}
	return c;
}

/* ************************************************************************* */
double Constrained::distance(const Vector& v) const {
	Vector w = Diagonal::whiten(v); // get noisemodel for constrained elements
	// TODO Find a better way of doing these checks
	for (size_t i=0; i<dim_; ++i) { // add mu weights on constrained variables
		if (isinf(w[i])) // whiten makes constrained variables infinite
			w[i] = v[i] * sqrt(mu_[i]); // TODO: may want to store sqrt rather than rebuild
		if (isnan(w[i])) // ensure no other invalid values make it through
			w[i] = v[i];
	}
	return w.dot(w);
}

/* ************************************************************************* */
Matrix Constrained::Whiten(const Matrix& H) const {
	// selective scaling
	return vector_scale(invsigmas(), H, true);
}

/* ************************************************************************* */
void Constrained::WhitenInPlace(Matrix& H) const {
	// selective scaling
	vector_scale_inplace(invsigmas(), H, true);
}

/* ************************************************************************* */
Constrained::shared_ptr Constrained::unit() const {
	Vector sigmas = ones(dim());
	for (size_t i=0; i<dim(); ++i)
		if (this->sigmas_(i) == 0.0)
			sigmas(i) = 0.0;
	return MixedSigmas(mu_, sigmas);
}

/* ************************************************************************* */
// Special version of QR for Constrained calls slower but smarter code
// that deals with possibly zero sigmas
// It is Gram-Schmidt orthogonalization rather than Householder
// Previously Diagonal::QR
SharedDiagonal Constrained::QR(Matrix& Ab) const {
	bool verbose = false;
	if (verbose) cout << "\nStarting Constrained::QR" << endl;

	// get size(A) and maxRank
	size_t m = Ab.rows(), n = Ab.cols()-1;
	size_t maxRank = min(m,n);

	// create storage for [R d]
	typedef boost::tuple<size_t, Vector, double> Triple;
	list<Triple> Rd;

	Vector pseudo(m); // allocate storage for pseudo-inverse
	Vector invsigmas = reciprocal(sigmas_);
	Vector weights = emul(invsigmas,invsigmas); // calculate weights once

	// We loop over all columns, because the columns that can be eliminated
	// are not necessarily contiguous. For each one, estimate the corresponding
	// scalar variable x as d-rS, with S the separator (remaining columns).
	// Then update A and b by substituting x with d-rS, zero-ing out x's column.
	for (size_t j=0; j<n; ++j) {
		// extract the first column of A
		Vector a = Ab.col(j);

		// Calculate weighted pseudo-inverse and corresponding precision
		tic(1, "constrained_QR weightedPseudoinverse");
		double precision = weightedPseudoinverse(a, weights, pseudo);
		toc(1, "constrained_QR weightedPseudoinverse");

		// If precision is zero, no information on this column
		// This is actually not limited to constraints, could happen in Gaussian::QR
		// In that case, we're probably hosed. TODO: make sure Householder is rank-revealing
		if (precision < 1e-8) continue;

		tic(2, "constrained_QR create rd");
		// create solution [r d], rhs is automatically r(n)
		Vector rd(n+1); // uninitialized !
		rd(j)=1.0; // put 1 on diagonal
		for (size_t j2=j+1; j2<n+1; ++j2) // and fill in remainder with dot-products
			rd(j2) = pseudo.dot(Ab.col(j2));
		toc(2, "constrained_QR create rd");

		// construct solution (r, d, sigma)
		Rd.push_back(boost::make_tuple(j, rd, precision));

		// exit after rank exhausted
		if (Rd.size()>=maxRank) break;

		// update Ab, expensive, using outer product
		tic(3, "constrained_QR update Ab");
		Ab.middleCols(j+1,n-j) -= a * rd.segment(j+1, n-j).transpose();
		toc(3, "constrained_QR update Ab");
	}

	// Create storage for precisions
	Vector precisions(Rd.size());

	tic(4, "constrained_QR write back into Ab");
	// Write back result in Ab, imperative as we are
	// TODO: test that is correct if a column was skipped !!!!
	size_t i = 0; // start with first row
	bool mixed = false;
	BOOST_FOREACH(const Triple& t, Rd) {
		const size_t& j  = t.get<0>();
		const Vector& rd = t.get<1>();
		precisions(i)    = t.get<2>();
		if (precisions(i)==inf) mixed = true;
		for (size_t j2=0; j2<j; ++j2)
			Ab(i,j2) = 0.0; // fill in zeros below diagonal anway
		for (size_t j2=j; j2<n+1; ++j2)
			Ab(i,j2) = rd(j2);
		i+=1;
	}
	toc(4, "constrained_QR write back into Ab");

	// Must include mu, as the defaults might be higher, resulting in non-convergence
	return mixed ? Constrained::MixedPrecisions(mu_, precisions) : Diagonal::Precisions(precisions);
}

/* ************************************************************************* */
// Isotropic
/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Sigma(size_t dim, double sigma, bool smart)  {
	if (smart && fabs(sigma-1.0)<1e-9) return Unit::Create(dim);
	return shared_ptr(new Isotropic(dim, sigma));
}

/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Variance(size_t dim, double variance, bool smart)  {
	if (smart && fabs(variance-1.0)<1e-9) return Unit::Create(dim);
	return shared_ptr(new Isotropic(dim, sqrt(variance)));
}

/* ************************************************************************* */
void Isotropic::print(const string& name) const {
	cout << name << "isotropic sigma " << " " << sigma_ << endl;
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
// Unit
/* ************************************************************************* */
void Unit::print(const std::string& name) const {
	cout << name << "unit (" << dim_ << ") " << endl;
}

/* ************************************************************************* */
// M-Estimator
/* ************************************************************************* */

namespace MEstimator {

/** produce a weight vector according to an error vector and the implemented
 * robust function */
Vector Base::weight(const Vector &error) const {
  const size_t n = error.rows();
  Vector w(n);
  for ( size_t i = 0 ; i < n ; ++i )
    w(i) = weight(error(i));
  return w;
}

/** square root version of the weight function */
Vector Base::sqrtWeight(const Vector &error) const {
  const size_t n = error.rows();
  Vector w(n);
  for ( size_t i = 0 ; i < n ; ++i )
    w(i) = sqrtWeight(error(i));
  return w;
}


/** The following three functions reweight block matrices and a vector
 * according to their weight implementation */

/** Reweight n block matrices with one error vector */
void Base::reweight(vector<Matrix> &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    BOOST_FOREACH(Matrix& Aj, A) {
      Aj *= w;
    }
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    BOOST_FOREACH(Matrix& Aj, A) {
      vector_scale_inplace(W,Aj);
    }
    error = emul(W, error);
  }
}

/** Reweight one block matrix with one error vector */
void Base::reweight(Matrix &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A);
    error = emul(W, error);
  }
}

/** Reweight two block matrix with one error vector */
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
    error = emul(W, error);
  }
}

/** Reweight three block matrix with one error vector */
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
    error = emul(W, error);
  }
}

/* ************************************************************************* */
// Null model
/* ************************************************************************* */

void Null::print(const std::string &s="") const
{ cout << s << "null ()" << endl; }

Null::shared_ptr Null::Create()
{ return shared_ptr(new Null()); }

Fair::Fair(const double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
  if ( c_ <= 0 ) {
    cout << "MEstimator Fair takes only positive double in constructor. forced to 1.0" << endl;
    c_ = 1.0;
  }
}

/* ************************************************************************* */
// Fair
/* ************************************************************************* */

double Fair::weight(const double &error) const
{ return 1.0 / (1.0 + fabs(error)/c_); }

void Fair::print(const std::string &s="") const
{ cout << s << "fair (" << c_ << ")" << endl; }

bool Fair::equals(const Base &expected, const double tol) const {
  const Fair* p = dynamic_cast<const Fair*> (&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_ ) < tol;
}

Fair::shared_ptr Fair::Create(const double c, const ReweightScheme reweight)
{ return shared_ptr(new Fair(c, reweight)); }

/* ************************************************************************* */
// Huber
/* ************************************************************************* */

Huber::Huber(const double k, const ReweightScheme reweight)
  : Base(reweight), k_(k) {
  if ( k_ <= 0 ) {
    cout << "MEstimator Huber takes only positive double in constructor. forced to 1.0" << endl;
    k_ = 1.0;
  }
}

double Huber::weight(const double &error) const {
	return (error < k_) ? (1.0) : (k_ / fabs(error));
}

void Huber::print(const std::string &s="") const {
	cout << s << "huber (" << k_ << ")" << endl;
}

bool Huber::equals(const Base &expected, const double tol) const {
	const Huber* p = dynamic_cast<const Huber*>(&expected);
	if (p == NULL) return false;
	return fabs(k_ - p->k_) < tol;
}

Huber::shared_ptr Huber::Create(const double c, const ReweightScheme reweight) {
	return shared_ptr(new Huber(c, reweight));
}

} // namespace MEstimator

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
