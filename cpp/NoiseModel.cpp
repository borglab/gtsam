/*
 * NoiseModel
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#include <limits>
#include <iostream>
#include <typeinfo>
#include <stdexcept>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>
#include "NoiseModel.h"

namespace ublas = boost::numeric::ublas;
typedef ublas::matrix_column<Matrix> column;

static double inf = std::numeric_limits<double>::infinity();
using namespace std;

namespace gtsam {
	namespace noiseModel {

		/* ************************************************************************* */
		Gaussian::shared_ptr Gaussian::Covariance(const Matrix& covariance, bool smart) {
			size_t m = covariance.size1(), n = covariance.size2();
			if (m != n) throw invalid_argument("Gaussian::Covariance: covariance not square");
			if (smart) {
				// check all non-diagonal entries
				int i,j;
				for (i = 0; i < m; i++)
					for (j = 0; j < n; j++)
						if (i != j && fabs(covariance(i, j) > 1e-9)) goto full;
				Vector variances(n);
				for (j = 0; j < n; j++) variances(j) = covariance(j,j);
				return Diagonal::Variances(variances,true);
			}
			full: return shared_ptr(new Gaussian(inverse_square_root(covariance)));
		}

    void Gaussian::print(const string& name) const {
			gtsam::print(sqrt_information_, "Gaussian");
		}

		bool Gaussian::equals(const Base& expected, double tol) const {
			const Gaussian* p = dynamic_cast<const Gaussian*> (&expected);
			if (p == NULL) return false;
			if (typeid(*this) != typeid(*p)) return false;
			return equal_with_abs_tol(sqrt_information_, p->sqrt_information_, sqrt(tol));
		}

		Vector Gaussian::whiten(const Vector& v) const {
			return sqrt_information_ * v;
		}

		Vector Gaussian::unwhiten(const Vector& v) const {
			return backSubstituteUpper(sqrt_information_, v);
		}

		double Gaussian::Mahalanobis(const Vector& v) const {
			// Note: for Diagonal, which does ediv_, will be correct for constraints
			Vector w = whiten(v);
			return inner_prod(w, w);
		}

		Matrix Gaussian::Whiten(const Matrix& H) const {
		  return sqrt_information_ * H;
		}

		void Gaussian::WhitenInPlace(Matrix& H) const {
		  H = sqrt_information_ * H;
		}

		// General QR, see also special version in Constrained
		sharedDiagonal Gaussian::QR(Matrix& Ab) const {
			// get size(A) and maxRank
			// TODO: really no rank problems ?
			size_t m = Ab.size1(), n = Ab.size2()-1;
			size_t maxRank = min(m,n);

			// pre-whiten everything (cheaply if possible)
			WhitenInPlace(Ab);

			// Perform in-place Householder
			// TODO: think about 1 on diagonal.
			// Currently, GaussianConditional assumes unit upper
			householder_(Ab, maxRank);

			return Unit::Create(maxRank);
		}

		/* ************************************************************************* */
		// TODO: can we avoid calling reciprocal twice ?
		Diagonal::Diagonal(const Vector& sigmas) :
					Gaussian(diag(reciprocal(sigmas))), invsigmas_(reciprocal(sigmas)),
							sigmas_(sigmas) {
				}

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

    void Diagonal::print(const string& name) const {
			gtsam::print(sigmas_, "Diagonal sigmas " + name);
		}

		Vector Diagonal::whiten(const Vector& v) const {
			return emul(v, invsigmas_);
		}

		Vector Diagonal::unwhiten(const Vector& v) const {
			return emul(v, sigmas_);
		}

		Matrix Diagonal::Whiten(const Matrix& H) const {
		  return vector_scale(invsigmas_, H);
		}

		void Diagonal::WhitenInPlace(Matrix& H) const {
		  H = vector_scale(invsigmas_, H);
		}

		/* ************************************************************************* */

		void Constrained::print(const std::string& name) const {
			gtsam::print(sigmas_, "Constrained sigmas " + name);
		}

		Vector Constrained::whiten(const Vector& v) const {
			// ediv_ does the right thing with the errors
			return ediv_(v, sigmas_);
		}

		Matrix Constrained::Whiten(const Matrix& H) const {
			throw logic_error("noiseModel::Constrained cannot Whiten");
		}

		void Constrained::WhitenInPlace(Matrix& H) const {
			throw logic_error("noiseModel::Constrained cannot Whiten");
		}

		/* ************************************************************************* */
		// update A, b
		// A' \define A_{S}-ar and b'\define b-ad
		// Linear algebra: takes away projection on latest orthogonal
		// Graph: make a new factor on the separator S
		// __attribute__ ((noinline))	// uncomment to prevent inlining when profiling
		static void updateAb(Matrix& Ab, int j, const Vector& a, const Vector& rd) {
			size_t m = Ab.size1(), n = Ab.size2()-1;
			for (int i = 0; i < m; i++) { // update all rows
				double ai = a(i);
				double *Aij = Ab.data().begin() + i * (n+1) + j + 1;
				const double *rptr = rd.data().begin() + j + 1;
				// Ab(i,j+1:end) -= ai*rd(j+1:end)
				for (int j2 = j + 1; j2 < n+1; j2++, Aij++, rptr++)
					*Aij -= ai * (*rptr);
			}
		}

		// Special version of QR for Constrained calls slower but smarter code
		// that deals with possibly zero sigmas
		// It is Gram-Schmidt orthogonalization rather than Householder
		sharedDiagonal Constrained::QR(Matrix& Ab) const {
			// get size(A) and maxRank
			size_t m = Ab.size1(), n = Ab.size2()-1;
			size_t maxRank = min(m,n);

			// create storage for [R d]
			typedef boost::tuple<size_t, Vector, double> Triple;
			list<Triple> Rd;

			Vector pseudo(m); // allocate storage for pseudo-inverse
			Vector weights = emul(invsigmas_,invsigmas_); // calculate weights once

			// We loop over all columns, because the columns that can be eliminated
			// are not necessarily contiguous. For each one, estimate the corresponding
			// scalar variable x as d-rS, with S the separator (remaining columns).
			// Then update A and b by substituting x with d-rS, zero-ing out x's column.
			for (size_t j=0; j<n; ++j) {
				// extract the first column of A
				// ublas::matrix_column is slower ! TODO Really, why ????
				//  AGC: if you use column() you will automatically call ublas, use
				//      column_() to actually use the one in our library
				Vector a(column(Ab, j));

				// Calculate weighted pseudo-inverse and corresponding precision
				double precision = weightedPseudoinverse(a, weights, pseudo);

				// If precision is zero, no information on this column
				// This is actually not limited to constraints, could happen in Gaussian::QR
				// In that case, we're probably hosed. TODO: make sure Householder is rank-revealing
				if (precision < 1e-8) continue;

				// create solution [r d], rhs is automatically r(n)
				Vector rd(n+1); // uninitialized !
				rd(j)=1.0; // put 1 on diagonal
				for (size_t j2=j+1; j2<n+1; ++j2) // and fill in remainder with dot-products
					rd(j2) = inner_prod(pseudo, ublas::matrix_column<Matrix>(Ab, j2));

				// construct solution (r, d, sigma)
				Rd.push_back(boost::make_tuple(j, rd, precision));

				// exit after rank exhausted
				if (Rd.size()>=maxRank) break;

				// update Ab, expensive, using outer product
				updateAb(Ab, j, a, rd);
			}

			// Create storage for precisions
			Vector precisions(Rd.size());

			// Write back result in Ab, imperative as we are
			// TODO: test that is correct if a column was skipped !!!!
			size_t i = 0; // start with first row
			BOOST_FOREACH(const Triple& t, Rd) {
				const size_t& j  = t.get<0>();
				const Vector& rd = t.get<1>();
				precisions(i)    = t.get<2>();
				for (size_t j2=0; j2<j; ++j2) Ab(i,j2) = 0.0; // fill in zeros below diagonal anway
				for (size_t j2=j; j2<n+1; ++j2) // copy the j-the row TODO memcpy
					Ab(i,j2) = rd(j2);
				i+=1;
			}

			return Diagonal::Precisions(precisions);
		}

		/* ************************************************************************* */

    Isotropic::shared_ptr Isotropic::Variance(size_t dim, double variance, bool smart)  {
    	if (smart && fabs(variance-1.0)<1e-9) return Unit::Create(dim);
    	return shared_ptr(new Isotropic(dim, sqrt(variance)));
    }

    void Isotropic::print(const string& name) const {
			cout << "Isotropic sigma " << name << " " << sigma_ << endl;
		}

		double Isotropic::Mahalanobis(const Vector& v) const {
			double dot = inner_prod(v, v);
			return dot * invsigma_ * invsigma_;
		}

		Vector Isotropic::whiten(const Vector& v) const {
			return v * invsigma_;
		}

		Vector Isotropic::unwhiten(const Vector& v) const {
			return v * sigma_;
		}

		Matrix Isotropic::Whiten(const Matrix& H) const {
		  return invsigma_ * H;
		}

		void Isotropic::WhitenInPlace(Matrix& H) const {
		  H *= invsigma_;
		}

		/* ************************************************************************* */
		void Unit::print(const std::string& name) const {
			cout << "Unit (" << dim_ << ") " << name << endl;
		}

	/* ************************************************************************* */

	}
} // gtsam
