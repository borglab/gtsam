/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NoiseModel.h
 * @date  Jan 13, 2010
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#pragma once

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/optional.hpp>
#include <gtsam/base/Matrix.h>
#include <cmath>

namespace gtsam {

	/// All noise models live in the noiseModel namespace
	namespace noiseModel {

		// Forward declaration
		class Gaussian;
		class Diagonal;
		class Constrained;
		class Isotropic;
		class Unit;

		//---------------------------------------------------------------------------------------

		/**
		 * noiseModel::Base is the abstract base class for all noise models.
		 *
		 * Noise models must implement a 'whiten' function to normalize an error vector,
		 * and an 'unwhiten' function to unnormalize an error vector.
		 */
		class Base {

		public:
      typedef boost::shared_ptr<Base> shared_ptr;

		protected:

			size_t dim_;

		public:

			/** primary constructor @param dim is the dimension of the model */
			Base(size_t dim = 1):dim_(dim) {}
			virtual ~Base() {}

			/// Dimensionality
			inline size_t dim() const { return dim_;}

      virtual void print(const std::string& name = "") const = 0;

      virtual bool equals(const Base& expected, double tol=1e-9) const = 0;

			/**
			 * Whiten an error vector.
			 */
			virtual Vector whiten(const Vector& v) const = 0;

			/**
			 * Unwhiten an error vector.
			 */
			virtual Vector unwhiten(const Vector& v) const = 0;

      virtual double distance(const Vector& v) const = 0;

      virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const = 0;
      virtual void WhitenSystem(Matrix& A, Vector& b) const = 0;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const = 0;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const = 0;

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

		//---------------------------------------------------------------------------------------

		/**
		 * Gaussian implements the mathematical model
		 *  |R*x|^2 = |y|^2 with R'*R=inv(Sigma)
		 * where
		 *   y = whiten(x) = R*x
		 *   x = unwhiten(x) = inv(R)*y
		 * as indeed
		 *   |y|^2 = y'*y = x'*R'*R*x
		 * Various derived classes are available that are more efficient.
		 * The named constructors return a shared_ptr because, when the smart flag is true,
		 * the underlying object might be a derived class such as Diagonal.
		 */
		class Gaussian: public Base {

		protected:

			/** Matrix square root of information matrix (R) */
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
			 */
			static shared_ptr SqrtInformation(const Matrix& R) {
				return shared_ptr(new Gaussian(R.rows(),R));
			}

			/**
			 * A Gaussian noise model created by specifying a covariance matrix.
			 * @param covariance The square covariance Matrix
			 * @param smart check if can be simplified to derived class
			 */
			static shared_ptr Covariance(const Matrix& covariance, bool smart = true);

			virtual void print(const std::string& name) const;
			virtual bool equals(const Base& expected, double tol=1e-9) const;
			virtual Vector whiten(const Vector& v) const;
			virtual Vector unwhiten(const Vector& v) const;

			/**
			 * Mahalanobis distance v'*R'*R*v = <R*v,R*v>
			 */
			virtual double Mahalanobis(const Vector& v) const;

      inline virtual double distance(const Vector& v) const {
        return Mahalanobis(v);
      }

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
      virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const ;
			virtual void WhitenSystem(Matrix& A, Vector& b) const ;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const ;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

			/**
			 * Apply appropriately weighted QR factorization to the system [A b]
			 *               Q'  *   [A b]  =  [R d]
			 * Dimensions: (r*m) * m*(n+1) = r*(n+1)
			 * @param Ab is the m*(n+1) augmented system matrix [A b]
			 * @return in-place QR factorization [R d]. Below-diagonal is undefined !!!!!
			 */
			virtual boost::shared_ptr<Diagonal> QR(Matrix& Ab) const;

			/**
			 * Return R itself, but note that Whiten(H) is cheaper than R*H
			 */
			virtual Matrix R() const { return thisR();}

			/**
			 * Simple check for constrained-ness
			 * FIXME Find a better way of handling this
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

		//---------------------------------------------------------------------------------------

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

			boost::optional<Vector> invsigmas_; ///< optional to allow for constraints

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
			static shared_ptr Sigmas(const Vector& sigmas, bool smart = true);

			/**
			 * A diagonal noise model created by specifying a Vector of variances, i.e.
			 * i.e. the diagonal of the covariance matrix.
			 * @param variances A vector containing the variances of this noise model
			 * @param smart check if can be simplified to derived class
			 */
			static shared_ptr Variances(const Vector& variances, bool smart = true);

			/**
			 * A diagonal noise model created by specifying a Vector of precisions, i.e.
			 * i.e. the diagonal of the information matrix, i.e., weights
			 */
			static shared_ptr Precisions(const Vector& precisions, bool smart = true) {
				return Variances(reciprocal(precisions), smart);
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

		//---------------------------------------------------------------------------------------

		/**
		 * A Constrained constrained model is a specialization of Diagonal which allows
		 * some or all of the sigmas to be zero, forcing the error to be zero there.
		 * All other Gaussian models are guaranteed to have a non-singular square-root
		 * information matrix, but this class is specifically equipped to deal with
		 * singular noise models, specifically: whiten will return zero on those
		 * components that have zero sigma *and* zero error, unchanged otherwise.
		 *
		 * While a hard constraint may seem to be a case in which there is infinite error,
		 * we do not ever produce an error value of infinity to allow for constraints
		 * to actually be optimized rather than self-destructing if not initialized correctly.
		 *
		 * The distance function in this function provides an error model
		 * for a penalty function with a scaling function, assuming a mask of
		 */
		class Constrained : public Diagonal {
		protected:

			// Sigmas are contained in the base class

			// Penalty function parameters
			Vector mu_;

			/** protected constructor takes sigmas */
			// Keeps only sigmas and calculates invsigmas when necessary
			Constrained(const Vector& sigmas = zero(1)) :
				Diagonal(sigmas, false), mu_(repeat(sigmas.size(), 1000.0)) {}

			// Keeps only sigmas and calculates invsigmas when necessary
			// allows for specifying mu
			Constrained(const Vector& mu, const Vector& sigmas) :
				Diagonal(sigmas, false), mu_(mu) {}

		public:

			typedef boost::shared_ptr<Constrained> shared_ptr;

			virtual ~Constrained() {}

			/// Access mu as a vector
			const Vector& mu() const { return mu_; }

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 */
			static shared_ptr MixedSigmas(const Vector& mu, const Vector& sigmas,
					bool smart = true);

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 */
			static shared_ptr MixedSigmas(const Vector& sigmas, bool smart = true) {
				return MixedSigmas(repeat(sigmas.size(), 1000.0), sigmas, smart);
			}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 */
			static shared_ptr MixedSigmas(double m, const Vector& sigmas,
					bool smart = true) {
				return MixedSigmas(repeat(sigmas.size(), m), sigmas, smart);
			}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * standard devations, some of which might be zero
			 */
			static shared_ptr MixedVariances(const Vector& mu, const Vector& variances) {
				return shared_ptr(new Constrained(mu, esqrt(variances)));
			}
			static shared_ptr MixedVariances(const Vector& variances) {
				return shared_ptr(new Constrained(esqrt(variances)));
			}

			/**
			 * A diagonal noise model created by specifying a Vector of
			 * precisions, some of which might be inf
			 * TODO: allow for mu
			 */
			static shared_ptr MixedPrecisions(const Vector& mu, const Vector& precisions) {
				return MixedVariances(mu, reciprocal(precisions));
			}
			static shared_ptr MixedPrecisions(const Vector& precisions) {
				return MixedVariances(reciprocal(precisions));
			}

			/**
			 * The distance function for a constrained noisemodel,
			 * for non-constrained versions, uses sigmas, otherwise
			 * uses the penalty function with mu
			 */
      virtual double distance(const Vector& v) const;

			/** Fully constrained variations */
			static shared_ptr All(size_t dim) {
				return shared_ptr(new Constrained(repeat(dim, 1000.0), repeat(dim,0)));
			}

			/** Fully constrained variations */
			static shared_ptr All(size_t dim, const Vector& mu) {
				return shared_ptr(new Constrained(mu, repeat(dim,0)));
			}

			/** Fully constrained variations */
			static shared_ptr All(size_t dim, double m) {
				return shared_ptr(new Constrained(repeat(dim, m), repeat(dim,0)));
			}

			virtual void print(const std::string& name) const;

			/// Calculates error vector with weights applied
			virtual Vector whiten(const Vector& v) const;

			/// Whitening functions will perform partial whitening on rows
			/// with a non-zero sigma.  Other rows remain untouched.
			virtual Matrix Whiten(const Matrix& H) const;
			virtual void WhitenInPlace(Matrix& H) const;

			/**
			 * Apply QR factorization to the system [A b], taking into account constraints
			 */
			virtual Diagonal::shared_ptr QR(Matrix& Ab) const;

			/**
			 * Check constrained is always true
			 * FIXME Find a better way of handling this
			 */
			virtual bool isConstrained() const {return true;}

			/**
			 * Returns a Unit version of a constrained noisemodel in which
			 * constrained sigmas remain constrained and the rest are unit scaled
			 */
			shared_ptr unit() const;

		private:
			/** Serialization function */
			friend class boost::serialization::access;
			template<class ARCHIVE>
			void serialize(ARCHIVE & ar, const unsigned int version) {
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Diagonal);
				ar & BOOST_SERIALIZATION_NVP(mu_);
			}

		}; // Constrained

		//---------------------------------------------------------------------------------------

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
			static shared_ptr Sigma(size_t dim, double sigma, bool smart = true);

			/**
			 * An isotropic noise model created by specifying a variance = sigma^2.
			 * @param dim The dimension of this noise model
			 * @param variance The variance of this noise model
			 * @param smart check if can be simplified to derived class
			 */
			static shared_ptr Variance(size_t dim, double variance, bool smart = true);

			/**
			 * An isotropic noise model created by specifying a precision
			 */
			static shared_ptr Precision(size_t dim, double precision, bool smart = true)  {
				return Variance(dim, 1.0/precision, smart);
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

		//---------------------------------------------------------------------------------------

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

		// TODO: should not really exist
		/// The MEstimator namespace contains all robust error functions (not models)
		namespace MEstimator {

		//---------------------------------------------------------------------------------------

		class Base {
    public:
      enum ReweightScheme { Scalar, Block };
      typedef boost::shared_ptr<Base> shared_ptr;

		protected:
      /** the rows can be weighted independently accordint to the error
       * or uniformly with the norm of the right hand side */
      ReweightScheme reweight_;

    public:
      Base(): reweight_(Block) {}
		  Base(const ReweightScheme reweight):reweight_(reweight) {}
		  virtual ~Base() {}

		  /// robust error function to implement
		  virtual double weight(const double &error) const = 0;

		  virtual void print(const std::string &s) const = 0;
		  virtual bool equals(const Base& expected, const double tol=1e-8) const = 0;

		  inline double sqrtWeight(const double &error) const
		  { return std::sqrt(weight(error)); }

      /** produce a weight vector according to an error vector and the implemented
       * robust function */
      Vector weight(const Vector &error) const;

      /** square root version of the weight function */
      Vector sqrtWeight(const Vector &error) const;

      /** reweight block matrices and a vector according to their weight implementation */
      void reweight(std::vector<Matrix> &A, Vector &error) const;
		  void reweight(Matrix &A, Vector &error) const;
		  void reweight(Matrix &A1, Matrix &A2, Vector &error) const;
		  void reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const;
		};

		/// Null class is not robust so is a Gaussian ?
    class Null : public Base {
    public:
      typedef boost::shared_ptr<Null> shared_ptr;
      Null(const ReweightScheme reweight = Block) : Base(reweight) {}
      virtual ~Null() {}
      virtual double weight(const double &error) const { return 1.0; }
      virtual void print(const std::string &s) const;
      virtual bool equals(const Base& expected, const double tol=1e-8) const { return true; }
      static shared_ptr Create() ;
    };

    /// Fair implements the "Fair" robust error model (Zhang97ivc)
		class Fair : public Base {
		public:
		  typedef boost::shared_ptr<Fair> shared_ptr;
		protected:
		  double c_;
		public:
		  Fair(const double c, const ReweightScheme reweight = Block);
		  virtual ~Fair() {}
		  virtual double weight(const double &error) const ;
      virtual void print(const std::string &s) const ;
      virtual bool equals(const Base& expected, const double tol=1e-8) const ;
      static shared_ptr Create(const double c, const ReweightScheme reweight = Block) ;
		private:
      Fair(){}
		};

    /// Huber implements the "Huber" robust error model (Zhang97ivc)
	  class Huber : public Base {
	    public:
	      typedef boost::shared_ptr<Huber> shared_ptr;
	    protected:
	      double k_;
	    public:
	      Huber(const double k, const ReweightScheme reweight = Block);
	      virtual ~Huber() {}
	      virtual double weight(const double &error) const ;
	      virtual void print(const std::string &s) const ;
	      virtual bool equals(const Base& expected, const double tol=1e-8) const ;
	      static shared_ptr Create(const double k, const ReweightScheme reweight = Block) ;
	    private:
	      Huber(){}
	    };

		} ///\namespace MEstimator

		/// Base class for robust error models
		class Robust : public Base {
		public:
		  typedef boost::shared_ptr<Robust> shared_ptr;

		protected:
      typedef MEstimator::Base RobustModel;
		  typedef noiseModel::Base NoiseModel;

      const RobustModel::shared_ptr robust_; ///< robust error function used
      const NoiseModel::shared_ptr noise_;   ///< noise model used

		public:

      /// Constructor
      Robust(const RobustModel::shared_ptr robust, const NoiseModel::shared_ptr noise)
      : Base(noise->dim()), robust_(robust), noise_(noise) {}

      /// Destructor
      virtual ~Robust() {}

      virtual void print(const std::string& name) const;
      virtual bool equals(const Base& expected, double tol=1e-9) const;

      /// Return the contained robust error function
      const RobustModel::shared_ptr& robust() const { return robust_; }

      /// Return the contained noise model
      const NoiseModel::shared_ptr& noise() const { return noise_; }

      // TODO: all function below are dummy but necessary for the noiseModel::Base

      inline virtual Vector whiten(const Vector& v) const
      { return noise_->whiten(v); }
      inline virtual Vector unwhiten(const Vector& v) const
      { return noise_->unwhiten(v); }
      inline virtual double distance(const Vector& v) const
      { return noise_->distance(v); }

      // TODO: these are really robust iterated re-weighting support functions

      virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

      static shared_ptr Create(
        const RobustModel::shared_ptr &robust, const NoiseModel::shared_ptr noise);

		private:
      Robust();
		};

	} // namespace noiseModel

	/** Note, deliberately not in noiseModel namespace.
	 * Deprecated. Only for compatibility with previous version.
	 */
	typedef noiseModel::Base::shared_ptr SharedNoiseModel;
	typedef noiseModel::Gaussian::shared_ptr SharedGaussian;
	typedef noiseModel::Diagonal::shared_ptr SharedDiagonal;

} // namespace gtsam


