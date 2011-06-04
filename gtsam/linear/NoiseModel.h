/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * NoiseModel.h
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

	class SharedDiagonal; // forward declare, defined at end

	namespace noiseModel {

		class Gaussian;
		class Diagonal;
		class Constrained;
		class Isotropic;
		class Unit;

		/**
		 * noiseModel::Base is the abstract base class for all noise models.
		 *
		 * Noise models must implement a 'whiten' function to normalize an error vector,
		 * and an 'unwhiten' function to unnormalize an error vector.
		 */
		class Base : public Testable<Base> {

		protected:

			size_t dim_;

		public:

			/** primary constructor @param dim is the dimension of the model */
			Base(size_t dim = 1):dim_(dim) {}
			virtual ~Base() {}

			/**
			 * Dimensionality
			 */
			inline size_t dim() const { return dim_;}

			/**
			 * Whiten an error vector.
			 */
			virtual Vector whiten(const Vector& v) const = 0;

			/**
			 * Unwhiten an error vector.
			 */
			virtual Vector unwhiten(const Vector& v) const = 0;

			/** in-place whiten, override if can be done more efficiently */
			virtual void whitenInPlace(Vector& v) const {
				v = whiten(v);
			}

			/** in-place unwhiten, override if can be done more efficiently */
			virtual void unwhitenInPlace(Vector& v) const {
				v = unwhiten(v);
			}

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_NVP(dim_);
			}
		};

		/**
		 * Gaussian implements the mathematical model
		 *  |R*x|^2 = |y|^2 with R'*R=inv(Sigma)
		 * where
		 *   y = whiten(x) = R*x
		 *   x = unwhiten(x) = inv(R)*y
		 * as indeed
		 *   |y|^2 = y'*y = x'*R'*R*x
		 * Various derived classes are available that are more efficient.
		 */
		struct Gaussian: public Base {

		protected:

			// TODO: store as boost upper-triangular or whatever is passed from above
			/* Matrix square root of information matrix (R) */
			boost::optional<Matrix> sqrt_information_;

		private:

			/**
			 * Return R itself, but note that Whiten(H) is cheaper than R*H
			 */
			const Matrix& thisR() const {
				// should never happen
				if (!sqrt_information_) throw std::runtime_error("Gaussian: has no R matrix");
				return *sqrt_information_;
			}

		protected:

			/** protected constructor takes square root information matrix */
			Gaussian(size_t dim = 1, const boost::optional<Matrix>& sqrt_information = boost::none) :
				Base(dim), sqrt_information_(sqrt_information) {
			}

		public:

			typedef boost::shared_ptr<Gaussian> shared_ptr;

			virtual ~Gaussian() {}

			/**
			 * A Gaussian noise model created by specifying a square root information matrix.
			 * @param smart, check if can be simplified to derived class
			 */
			static shared_ptr SqrtInformation(const Matrix& R) {
				return shared_ptr(new Gaussian(R.rows(),R));
			}

			/**
			 * A Gaussian noise model created by specifying a covariance matrix.
			 * @param smart, check if can be simplified to derived class
			 */
			static shared_ptr Covariance(const Matrix& covariance, bool smart=false);

			virtual void print(const std::string& name) const;
			virtual bool equals(const Base& expected, double tol=1e-9) const;
			virtual Vector whiten(const Vector& v) const;
			virtual Vector unwhiten(const Vector& v) const;

			/**
			 * Mahalanobis distance v'*R'*R*v = <R*v,R*v>
			 */
			virtual double Mahalanobis(const Vector& v) const;

			/**
			 * Multiply a derivative with R (derivative of whiten)
			 * Equivalent to whitening each column of the input matrix.
			 */
			virtual Matrix Whiten(const Matrix& H) const;

			/**
			 * In-place version
			 */
			virtual void WhitenInPlace(Matrix& H) const;

			/**
			 * Whiten a system, in place as well
			 */
			inline void WhitenSystem(Matrix& A, Vector& b) const {
				WhitenInPlace(A);
				whitenInPlace(b);
			}

			/**
			 * Apply appropriately weighted QR factorization to the system [A b]
			 *               Q'  *   [A b]  =  [R d]
			 * Dimensions: (r*m) * m*(n+1) = r*(n+1)
			 * @param Ab is the m*(n+1) augmented system matrix [A b]
			 * @return in-place QR factorization [R d]. Below-diagonal is undefined !!!!!
			 */
			virtual SharedDiagonal QR(Matrix& Ab) const;
			// FIXME: these previously had firstZeroRows - what did this do?
//			virtual SharedDiagonal QRColumnWise(Matrix& Ab, std::vector<int>& firstZeroRows) const;
//			virtual SharedDiagonal QR(Matrix& Ab, boost::optional<std::vector<int>&> firstZeroRows = boost::none) const;

			/**
			 * Cholesky factorization
			 */
			virtual SharedDiagonal Cholesky(Matrix& Ab, size_t nFrontals) const;

			/**
			 * Return R itself, but note that Whiten(H) is cheaper than R*H
			 */
			virtual Matrix R() const { return thisR();}

			/**
			 * Simple check for constrained-ness
			 */
			virtual bool isConstrained() const {return false;}

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
				ar & BOOST_SERIALIZATION_NVP(sqrt_information_);
			}

		}; // Gaussian


		/**
		 * A diagonal noise model implements a diagonal covariance matrix, with the
		 * elements of the diagonal specified in a Vector.  This class has no public
		 * constructors, instead, use the static constructor functions Sigmas etc...
		 */
		class Diagonal : public Gaussian {
		protected:

			/** sigmas and reciprocal */
			Vector sigmas_;
		private:
			boost::optional<Vector> invsigmas_; /// optional to allow for constraints

		protected:
			/** protected constructor takes sigmas */
			Diagonal();

			/** constructor to allow for disabling initializaion of invsigmas */
			Diagonal(const Vector& sigmas, bool initialize_invsigmas=true);

		public:

			typedef boost::shared_ptr<Diagonal> shared_ptr;

			virtual ~Diagonal() {}

			/**
			 * A diagonal noise model created by specifying a Vector of sigmas, i.e.
			 * standard devations, the diagonal of the square root covariance matrix.
			 */
			static shared_ptr Sigmas(const Vector& sigmas, bool smart=false);

			/**
			 * A diagonal noise model created by specifying a Vector of variances, i.e.
			 * i.e. the diagonal of the covariance matrix.
			 * @param smart, check if can be simplified to derived class
			 */
			static shared_ptr Variances(const Vector& variances, bool smart = false);

			/**
			 * A diagonal noise model created by specifying a Vector of precisions, i.e.
			 * i.e. the diagonal of the information matrix, i.e., weights
			 */
			static shared_ptr Precisions(const Vector& precisions) {
				return Variances(reciprocal(precisions));
			}

			virtual void print(const std::string& name) const;
			virtual Vector whiten(const Vector& v) const;
			virtual Vector unwhiten(const Vector& v) const;
			virtual Matrix Whiten(const Matrix& H) const;
			virtual void WhitenInPlace(Matrix& H) const;

			/**
			 * Return standard deviations (sqrt of diagonal)
			 */
			inline const Vector& sigmas() const { return sigmas_; }
			inline double sigma(size_t i) const { return sigmas_(i); }

			/**
			 * Return sqrt precisions
			 */
			Vector invsigmas() const;
			double invsigma(size_t i) const;

			/**
			 * generate random variate
			 */
			virtual Vector sample() const;

			/**
			 * Return R itself, but note that Whiten(H) is cheaper than R*H
			 */
			virtual Matrix R() const {
				return diag(invsigmas());
			}

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Gaussian);
				ar & BOOST_SERIALIZATION_NVP(sigmas_);
				ar & BOOST_SERIALIZATION_NVP(invsigmas_);
			}
		}; // Diagonal

		/**
		 * A Constrained constrained model is a specialization of Diagonal which allows
		 * some or all of the sigmas to be zero, forcing the error to be zero there.
		 * All other Gaussian models are guaranteed to have a non-singular square-root
		 * information matrix, but this class is specifically equipped to deal with
		 * singular noise models, specifically: whiten will return zero on those
		 * components that have zero sigma *and* zero error, infinity otherwise.
		 */
		class Constrained : public Diagonal {
		protected:

			// Constrained does not have member variables
			// Instead (possibly zero) sigmas are stored in Diagonal Base class

			/** protected constructor takes sigmas */
			// Keeps only sigmas and calculates invsigmas when necessary
			Constrained(const Vector& sigmas = zero(1)) :
				Diagonal(sigmas, false) {}

		public:

			typedef boost::shared_ptr<Constrained> shared_ptr;

			virtual ~Constrained() {}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 * TODO: make smart - check for zeros
			 */
			static shared_ptr MixedSigmas(const Vector& sigmas, bool smart = false) {
				return shared_ptr(new Constrained(sigmas));
			}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 */
			static shared_ptr MixedVariances(const Vector& variances) {
				return shared_ptr(new Constrained(esqrt(variances)));
			}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * precisions, some of which might be inf
			 */
			static shared_ptr MixedPrecisions(const Vector& precisions) {
				return MixedVariances(reciprocal(precisions));
			}

			/**
			 * Fully constrained. TODO: subclass ?
			 */
			static shared_ptr All(size_t dim) {
				return MixedSigmas(repeat(dim,0));
			}

			virtual void print(const std::string& name) const;
			virtual Vector whiten(const Vector& v) const;

			// Whitening Jacobians does not make sense for possibly constrained
			// noise model and will throw an exception.

			virtual Matrix Whiten(const Matrix& H) const;
			virtual void WhitenInPlace(Matrix& H) const;

			/**
			 * Apply QR factorization to the system [A b], taking into account constraints
			 */
			virtual SharedDiagonal QR(Matrix& Ab) const;

			/**
			 * Check constrained is always true
			 */
			virtual bool isConstrained() const {return true;}

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Diagonal);
			}

		}; // Constrained

		/**
		 * An isotropic noise model corresponds to a scaled diagonal covariance
		 * To construct, use one of the static methods
		 */
		class Isotropic : public Diagonal {
		protected:
			double sigma_, invsigma_;

			/** protected constructor takes sigma */
			Isotropic(size_t dim, double sigma) :
				Diagonal(repeat(dim, sigma)),sigma_(sigma),invsigma_(1.0/sigma) {}

			/* dummy constructor to allow for serialization */
			Isotropic() : Diagonal(repeat(1, 1.0)),sigma_(1.0),invsigma_(1.0) {}

		public:

			virtual ~Isotropic() {}

			typedef boost::shared_ptr<Isotropic> shared_ptr;

			/**
			 * An isotropic noise model created by specifying a standard devation sigma
			 */
			static shared_ptr Sigma(size_t dim, double sigma) {
				return shared_ptr(new Isotropic(dim, sigma));
			}

			/**
			 * An isotropic noise model created by specifying a variance = sigma^2.
			 * @param smart, check if can be simplified to derived class
			 */
			static shared_ptr Variance(size_t dim, double variance, bool smart = false);

			/**
			 * An isotropic noise model created by specifying a precision
			 */
			static shared_ptr Precision(size_t dim, double precision)  {
				return Variance(dim, 1.0/precision);
			}

			virtual void print(const std::string& name) const;
			virtual double Mahalanobis(const Vector& v) const;
			virtual Vector whiten(const Vector& v) const;
			virtual Vector unwhiten(const Vector& v) const;
			virtual Matrix Whiten(const Matrix& H) const;
			virtual void WhitenInPlace(Matrix& H) const;

			/**
			 * Return standard deviation
			 */
			inline double sigma() const { return sigma_; }

			/**
			 * generate random variate
			 */
			virtual Vector sample() const;

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Diagonal);
				ar & BOOST_SERIALIZATION_NVP(sigma_);
				ar & BOOST_SERIALIZATION_NVP(invsigma_);
			}

		};

		/**
		 * Unit: i.i.d. unit-variance noise on all m dimensions.
		 */
		class Unit : public Isotropic {
		protected:

			Unit(size_t dim=1): Isotropic(dim,1.0) {}

		public:

			typedef boost::shared_ptr<Unit> shared_ptr;

			virtual ~Unit() {}

			/**
			 * Create a unit covariance noise model
			 */
			static shared_ptr Create(size_t dim) {
				return shared_ptr(new Unit(dim));
			}

			virtual void print(const std::string& name) const;
			virtual double Mahalanobis(const Vector& v) const {return v.dot(v); }
			virtual Vector whiten(const Vector& v) const { return v; }
			virtual Vector unwhiten(const Vector& v) const { return v; }
			virtual Matrix Whiten(const Matrix& H) const { return H; }
			virtual void WhitenInPlace(Matrix& H) const {}

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Isotropic);
			}
		};

	} // namespace noiseModel

} // namespace gtsam


