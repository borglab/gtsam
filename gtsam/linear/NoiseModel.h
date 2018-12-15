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

#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/optional.hpp>

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
    class GTSAM_EXPORT Base {

    public:
      typedef boost::shared_ptr<Base> shared_ptr;

    protected:

      size_t dim_;

    public:

      /// primary constructor @param dim is the dimension of the model
      Base(size_t dim = 1):dim_(dim) {}
      virtual ~Base() {}

      /// true if a constrained noise model, saves slow/clumsy dynamic casting
      virtual bool isConstrained() const { return false; } // default false

      /// true if a unit noise model, saves slow/clumsy dynamic casting
      virtual bool isUnit() const { return false; }  // default false

      /// Dimensionality
      inline size_t dim() const { return dim_;}

      virtual void print(const std::string& name = "") const = 0;

      virtual bool equals(const Base& expected, double tol=1e-9) const = 0;

      /// Calculate standard deviations
      virtual Vector sigmas() const;

      /// Whiten an error vector.
      virtual Vector whiten(const Vector& v) const = 0;

      /// Whiten a matrix.
      virtual Matrix Whiten(const Matrix& H) const = 0;

      /// Unwhiten an error vector.
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

      /** in-place whiten, override if can be done more efficiently */
      virtual void whitenInPlace(Eigen::Block<Vector>& v) const {
        v = whiten(v);
      }

      /** in-place unwhiten, override if can be done more efficiently */
      virtual void unwhitenInPlace(Eigen::Block<Vector>& v) const {
        v = unwhiten(v);
      }

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
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
    class GTSAM_EXPORT Gaussian: public Base {

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
       * @param R The (upper-triangular) square root information matrix
       * @param smart check if can be simplified to derived class
       */
      static shared_ptr SqrtInformation(const Matrix& R, bool smart = true);

      /**
       * A Gaussian noise model created by specifying an information matrix.
       * @param M The information matrix
       * @param smart check if can be simplified to derived class
       */
      static shared_ptr Information(const Matrix& M, bool smart = true);

      /**
       * A Gaussian noise model created by specifying a covariance matrix.
       * @param covariance The square covariance Matrix
       * @param smart check if can be simplified to derived class
       */
      static shared_ptr Covariance(const Matrix& covariance, bool smart = true);

      virtual void print(const std::string& name) const;
      virtual bool equals(const Base& expected, double tol=1e-9) const;
      virtual Vector sigmas() const;
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
       * In-place version
       */
      virtual void WhitenInPlace(Eigen::Block<Matrix> H) const;

      /**
       * Whiten a system, in place as well
       */
      virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

      /**
       * Apply appropriately weighted QR factorization to the system [A b]
       *               Q'  *   [A b]  =  [R d]
       * Dimensions: (r*m) * m*(n+1) = r*(n+1), where r = min(m,n).
       * This routine performs an in-place factorization on Ab.
       * Below-diagonal elements are set to zero by this routine.
       * @param Ab is the m*(n+1) augmented system matrix [A b]
       * @return Empty SharedDiagonal() noise model: R,d are whitened
       */
      virtual boost::shared_ptr<Diagonal> QR(Matrix& Ab) const;

      /// Return R itself, but note that Whiten(H) is cheaper than R*H
      virtual Matrix R() const { return thisR();}

      /// Compute information matrix
      virtual Matrix information() const { return R().transpose() * R(); }

      /// Compute covariance matrix
      virtual Matrix covariance() const { return information().inverse(); }

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
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
    class GTSAM_EXPORT Diagonal : public Gaussian {
    protected:

      /**
       * Standard deviations (sigmas), their inverse and inverse square (weights/precisions)
       * These are all computed at construction: the idea is to use one shared model
       * where computation is done only once, the common use case in many problems.
       */
      Vector sigmas_, invsigmas_, precisions_;

    protected:
      /** protected constructor - no initializations */
      Diagonal();

      /** constructor to allow for disabling initialization of invsigmas */
      Diagonal(const Vector& sigmas);

    public:

      typedef boost::shared_ptr<Diagonal> shared_ptr;

      virtual ~Diagonal() {}

      /**
       * A diagonal noise model created by specifying a Vector of sigmas, i.e.
       * standard deviations, the diagonal of the square root covariance matrix.
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
        return Variances(precisions.array().inverse(), smart);
      }

      virtual void print(const std::string& name) const;
      virtual Vector sigmas() const { return sigmas_; }
      virtual Vector whiten(const Vector& v) const;
      virtual Vector unwhiten(const Vector& v) const;
      virtual Matrix Whiten(const Matrix& H) const;
      virtual void WhitenInPlace(Matrix& H) const;
      virtual void WhitenInPlace(Eigen::Block<Matrix> H) const;

      /**
       * Return standard deviations (sqrt of diagonal)
       */
      inline double sigma(size_t i) const { return sigmas_(i); }

      /**
       * Return sqrt precisions
       */
      inline const Vector& invsigmas() const { return invsigmas_; }
      inline double invsigma(size_t i) const {return invsigmas_(i);}

      /**
       * Return precisions
       */
      inline const Vector& precisions() const { return precisions_; }
      inline double precision(size_t i) const {return precisions_(i);}

      /**
       * Return R itself, but note that Whiten(H) is cheaper than R*H
       */
      virtual Matrix R() const {
        return invsigmas().asDiagonal();
      }

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
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
     */
    class GTSAM_EXPORT Constrained : public Diagonal {
    protected:

      // Sigmas are contained in the base class
      Vector mu_; ///< Penalty function weight - needs to be large enough to dominate soft constraints

      /**
       * protected constructor takes sigmas.
       * prevents any inf values
       * from appearing in invsigmas or precisions.
       * mu set to large default value (1000.0)
       */
      Constrained(const Vector& sigmas = Z_1x1);

      /**
       * Constructor that prevents any inf values
       * from appearing in invsigmas or precisions.
       * Allows for specifying mu.
       */
      Constrained(const Vector& mu, const Vector& sigmas);

    public:

      typedef boost::shared_ptr<Constrained> shared_ptr;

      virtual ~Constrained() {}

      /// true if a constrained noise mode, saves slow/clumsy dynamic casting
      virtual bool isConstrained() const { return true; }

      /// Return true if a particular dimension is free or constrained
      bool constrained(size_t i) const;

      /// Access mu as a vector
      const Vector& mu() const { return mu_; }

      /**
       * A diagonal noise model created by specifying a Vector of
       * standard devations, some of which might be zero
       */
      static shared_ptr MixedSigmas(const Vector& mu, const Vector& sigmas);

      /**
       * A diagonal noise model created by specifying a Vector of
       * standard devations, some of which might be zero
       */
      static shared_ptr MixedSigmas(const Vector& sigmas) {
        return MixedSigmas(Vector::Constant(sigmas.size(), 1000.0), sigmas);
      }

      /**
       * A diagonal noise model created by specifying a Vector of
       * standard devations, some of which might be zero
       */
      static shared_ptr MixedSigmas(double m, const Vector& sigmas) {
        return MixedSigmas(Vector::Constant(sigmas.size(), m), sigmas);
      }

      /**
       * A diagonal noise model created by specifying a Vector of
       * standard devations, some of which might be zero
       */
      static shared_ptr MixedVariances(const Vector& mu, const Vector& variances) {
        return shared_ptr(new Constrained(mu, variances.cwiseSqrt()));
      }
      static shared_ptr MixedVariances(const Vector& variances) {
        return shared_ptr(new Constrained(variances.cwiseSqrt()));
      }

      /**
       * A diagonal noise model created by specifying a Vector of
       * precisions, some of which might be inf
       */
      static shared_ptr MixedPrecisions(const Vector& mu, const Vector& precisions) {
        return MixedVariances(mu, precisions.array().inverse());
      }
      static shared_ptr MixedPrecisions(const Vector& precisions) {
        return MixedVariances(precisions.array().inverse());
      }

      /**
       * The distance function for a constrained noisemodel,
       * for non-constrained versions, uses sigmas, otherwise
       * uses the penalty function with mu
       */
      virtual double distance(const Vector& v) const;

      /** Fully constrained variations */
      static shared_ptr All(size_t dim) {
        return shared_ptr(new Constrained(Vector::Constant(dim, 1000.0), Vector::Constant(dim,0)));
      }

      /** Fully constrained variations */
      static shared_ptr All(size_t dim, const Vector& mu) {
        return shared_ptr(new Constrained(mu, Vector::Constant(dim,0)));
      }

      /** Fully constrained variations with a mu parameter */
      static shared_ptr All(size_t dim, double mu) {
        return shared_ptr(new Constrained(Vector::Constant(dim, mu), Vector::Constant(dim,0)));
      }

      virtual void print(const std::string& name) const;

      /// Calculates error vector with weights applied
      virtual Vector whiten(const Vector& v) const;

      /// Whitening functions will perform partial whitening on rows
      /// with a non-zero sigma.  Other rows remain untouched.
      virtual Matrix Whiten(const Matrix& H) const;
      virtual void WhitenInPlace(Matrix& H) const;
      virtual void WhitenInPlace(Eigen::Block<Matrix> H) const;

      /**
       * Apply QR factorization to the system [A b], taking into account constraints
       *               Q'  *   [A b]  =  [R d]
       * Dimensions: (r*m) * m*(n+1) = r*(n+1), where r = min(m,n).
       * This routine performs an in-place factorization on Ab.
       * Below-diagonal elements are set to zero by this routine.
       * @param Ab is the m*(n+1) augmented system matrix [A b]
       * @return diagonal noise model can be all zeros, mixed, or not-constrained
       */
      virtual Diagonal::shared_ptr QR(Matrix& Ab) const;

      /**
       * Returns a Unit version of a constrained noisemodel in which
       * constrained sigmas remain constrained and the rest are unit scaled
       */
      shared_ptr unit() const;

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Diagonal);
        ar & BOOST_SERIALIZATION_NVP(mu_);
      }

    }; // Constrained

    //---------------------------------------------------------------------------------------

    /**
     * An isotropic noise model corresponds to a scaled diagonal covariance
     * To construct, use one of the static methods
     */
    class GTSAM_EXPORT Isotropic : public Diagonal {
    protected:
      double sigma_, invsigma_;

      /** protected constructor takes sigma */
      Isotropic(size_t dim, double sigma) :
        Diagonal(Vector::Constant(dim, sigma)),sigma_(sigma),invsigma_(1.0/sigma) {}

      /* dummy constructor to allow for serialization */
      Isotropic() : Diagonal(Vector1::Constant(1.0)),sigma_(1.0),invsigma_(1.0) {}

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
      virtual void whitenInPlace(Vector& v) const;
      virtual void WhitenInPlace(Eigen::Block<Matrix> H) const;

      /**
       * Return standard deviation
       */
      inline double sigma() const { return sigma_; }

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Diagonal);
        ar & BOOST_SERIALIZATION_NVP(sigma_);
        ar & BOOST_SERIALIZATION_NVP(invsigma_);
      }

    };

    //---------------------------------------------------------------------------------------

    /**
     * Unit: i.i.d. unit-variance noise on all m dimensions.
     */
    class GTSAM_EXPORT Unit : public Isotropic {
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

      /// true if a unit noise model, saves slow/clumsy dynamic casting
      virtual bool isUnit() const { return true; }

      virtual void print(const std::string& name) const;
      virtual double Mahalanobis(const Vector& v) const {return v.dot(v); }
      virtual Vector whiten(const Vector& v) const { return v; }
      virtual Vector unwhiten(const Vector& v) const { return v; }
      virtual Matrix Whiten(const Matrix& H) const { return H; }
      virtual void WhitenInPlace(Matrix& /*H*/) const {}
      virtual void WhitenInPlace(Eigen::Block<Matrix> /*H*/) const {}
      virtual void whitenInPlace(Vector& /*v*/) const {}
      virtual void unwhitenInPlace(Vector& /*v*/) const {}
      virtual void whitenInPlace(Eigen::Block<Vector>& /*v*/) const {}
      virtual void unwhitenInPlace(Eigen::Block<Vector>& /*v*/) const {}

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Isotropic);
      }
    };

    /**
     * The mEstimator name space contains all robust error functions.
     * It mirrors the exposition at
     *  http://research.microsoft.com/en-us/um/people/zhang/INRIA/Publis/Tutorial-Estim/node24.html
     * which talks about minimizing \sum \rho(r_i), where \rho is a residual function of choice.
     *
     * To illustrate, let's consider the least-squares (L2), L1, and Huber estimators as examples:
     *
     * Name        Symbol          Least-Squares   L1-norm    Huber
     * Residual    \rho(x)         0.5*x^2         |x|        0.5*x^2 if x<k, 0.5*k^2 + k|x-k| otherwise
     * Derivative  \phi(x)         x               sgn(x)     x       if x<k, k sgn(x)         otherwise
     * Weight      w(x)=\phi(x)/x  1               1/|x|      1       if x<k, k/|x|            otherwise
     *
     * With these definitions, D(\rho(x), p) = \phi(x) D(x,p) = w(x) x D(x,p) = w(x) D(L2(x), p),
     * and hence we can solve the equivalent weighted least squares problem \sum w(r_i) \rho(r_i)
     *
     * Each M-estimator in the mEstimator name space simply implements the above functions.
     */
    namespace mEstimator {

      //---------------------------------------------------------------------------------------

      class GTSAM_EXPORT Base {
      public:
        enum ReweightScheme { Scalar, Block };
        typedef boost::shared_ptr<Base> shared_ptr;

      protected:
        /** the rows can be weighted independently according to the error
        * or uniformly with the norm of the right hand side */
        ReweightScheme reweight_;

      public:
        Base(const ReweightScheme reweight = Block):reweight_(reweight) {}
        virtual ~Base() {}

        /*
         * This method is responsible for returning the total penalty for a given amount of error.
         * For example, this method is responsible for implementing the quadratic function for an
         * L2 penalty, the absolute value function for an L1 penalty, etc.
         *
         * TODO(mike): There is currently a bug in GTSAM, where none of the mEstimator classes
         * implement a residual function, and GTSAM calls the weight function to evaluate the
         * total penalty, rather than calling the residual function. The weight function should be
         * used during iteratively reweighted least squares optimization, but should not be used to
         * evaluate the total penalty. The long-term solution is for all mEstimators to implement
         * both a weight and a residual function, and for GTSAM to call the residual function when
         * evaluating the total penalty. But for now, I'm leaving this residual method as pure
         * virtual, so the existing mEstimators can inherit this default fallback behavior.
         */
        virtual double residual(double error) const { return 0; };

        /*
         * This method is responsible for returning the weight function for a given amount of error.
         * The weight function is related to the analytic derivative of the residual function. See
         *  http://research.microsoft.com/en-us/um/people/zhang/INRIA/Publis/Tutorial-Estim/node24.html
         * for details. This method is required when optimizing cost functions with robust penalties
         * using iteratively re-weighted least squares.
         */
        virtual double weight(double error) const = 0;

        virtual void print(const std::string &s) const = 0;
        virtual bool equals(const Base& expected, double tol=1e-8) const = 0;

        double sqrtWeight(double error) const {
          return std::sqrt(weight(error));
        }

        /** produce a weight vector according to an error vector and the implemented
        * robust function */
        Vector weight(const Vector &error) const;

        /** square root version of the weight function */
        Vector sqrtWeight(const Vector &error) const {
          return weight(error).cwiseSqrt();
        }

        /** reweight block matrices and a vector according to their weight implementation */
        void reweight(Vector &error) const;
        void reweight(std::vector<Matrix> &A, Vector &error) const;
        void reweight(Matrix &A, Vector &error) const;
        void reweight(Matrix &A1, Matrix &A2, Vector &error) const;
        void reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_NVP(reweight_);
        }
      };

      /// Null class is not robust so is a Gaussian ?
      class GTSAM_EXPORT Null : public Base {
      public:
        typedef boost::shared_ptr<Null> shared_ptr;

        Null(const ReweightScheme reweight = Block) : Base(reweight) {}
        virtual ~Null() {}
        virtual double weight(double /*error*/) const { return 1.0; }
        virtual void print(const std::string &s) const;
        virtual bool equals(const Base& /*expected*/, double /*tol*/) const { return true; }
        static shared_ptr Create() ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        }
      };

      /// Fair implements the "Fair" robust error model (Zhang97ivc)
      class GTSAM_EXPORT Fair : public Base {
      protected:
        double c_;

      public:
        typedef boost::shared_ptr<Fair> shared_ptr;

        Fair(double c = 1.3998, const ReweightScheme reweight = Block);
        double weight(double error) const {
          return 1.0 / (1.0 + fabs(error) / c_);
        }
        void print(const std::string &s) const;
        bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double c, const ReweightScheme reweight = Block) ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(c_);
        }
      };

      /// Huber implements the "Huber" robust error model (Zhang97ivc)
      class GTSAM_EXPORT Huber : public Base {
      protected:
        double k_;

      public:
        typedef boost::shared_ptr<Huber> shared_ptr;

        Huber(double k = 1.345, const ReweightScheme reweight = Block);
        double weight(double error) const {
          return (error < k_) ? (1.0) : (k_ / fabs(error));
        }
        void print(const std::string &s) const;
        bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(k_);
        }
      };

      /// Cauchy implements the "Cauchy" robust error model (Lee2013IROS).  Contributed by:
      ///   Dipl.-Inform. Jan Oberlaender (M.Sc.), FZI Research Center for
      ///   Information Technology, Karlsruhe, Germany.
      ///   oberlaender@fzi.de
      /// Thanks Jan!
      class GTSAM_EXPORT Cauchy : public Base {
      protected:
        double k_, ksquared_;

      public:
        typedef boost::shared_ptr<Cauchy> shared_ptr;

        Cauchy(double k = 0.1, const ReweightScheme reweight = Block);
        double weight(double error) const {
          return ksquared_ / (ksquared_ + error*error);
        }
        void print(const std::string &s) const;
        bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(k_);
        }
      };

      /// Tukey implements the "Tukey" robust error model (Zhang97ivc)
      class GTSAM_EXPORT Tukey : public Base {
      protected:
        double c_, csquared_;

      public:
        typedef boost::shared_ptr<Tukey> shared_ptr;

        Tukey(double c = 4.6851, const ReweightScheme reweight = Block);
        double weight(double error) const {
          if (std::fabs(error) <= c_) {
            double xc2 = error*error/csquared_;
            return (1.0-xc2)*(1.0-xc2);
          }
          return 0.0;
        }
        void print(const std::string &s) const;
        bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(c_);
        }
      };

      /// Welsh implements the "Welsh" robust error model (Zhang97ivc)
      class GTSAM_EXPORT Welsh : public Base {
      protected:
        double c_, csquared_;

      public:
        typedef boost::shared_ptr<Welsh> shared_ptr;

        Welsh(double c = 2.9846, const ReweightScheme reweight = Block);
        double weight(double error) const {
          double xc2 = (error*error)/csquared_;
          return std::exp(-xc2);
        }
        void print(const std::string &s) const;
        bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(c_);
        }
      };

      /// GemanMcClure implements the "Geman-McClure" robust error model
      /// (Zhang97ivc).
      ///
      /// Note that Geman-McClure weight function uses the parameter c == 1.0,
      /// but here it's allowed to use different values, so we actually have
      /// the generalized Geman-McClure from (Agarwal15phd).
      class GTSAM_EXPORT GemanMcClure : public Base {
      public:
        typedef boost::shared_ptr<GemanMcClure> shared_ptr;

        GemanMcClure(double c = 1.0, const ReweightScheme reweight = Block);
        virtual ~GemanMcClure() {}
        virtual double weight(double error) const;
        virtual void print(const std::string &s) const;
        virtual bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      protected:
        double c_;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(c_);
        }
      };

      /// DCS implements the Dynamic Covariance Scaling robust error model
      /// from the paper Robust Map Optimization (Agarwal13icra).
      ///
      /// Under the special condition of the parameter c == 1.0 and not
      /// forcing the output weight s <= 1.0, DCS is similar to Geman-McClure.
      class GTSAM_EXPORT DCS : public Base {
      public:
        typedef boost::shared_ptr<DCS> shared_ptr;

        DCS(double c = 1.0, const ReweightScheme reweight = Block);
        virtual ~DCS() {}
        virtual double weight(double error) const;
        virtual void print(const std::string &s) const;
        virtual bool equals(const Base& expected, double tol=1e-8) const;
        static shared_ptr Create(double k, const ReweightScheme reweight = Block) ;

      protected:
        double c_;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(c_);
        }
      };

      /// L2WithDeadZone implements a standard L2 penalty, but with a dead zone of width 2*k,
      /// centered at the origin. The resulting penalty within the dead zone is always zero, and
      /// grows quadratically outside the dead zone. In this sense, the L2WithDeadZone penalty is
      /// "robust to inliers", rather than being robust to outliers. This penalty can be used to
      /// create barrier functions in a general way.
      class GTSAM_EXPORT L2WithDeadZone : public Base {
      protected:
          double k_;

      public:
          typedef boost::shared_ptr<L2WithDeadZone> shared_ptr;

          L2WithDeadZone(double k, const ReweightScheme reweight = Block);
          double residual(double error) const {
            const double abs_error = fabs(error);
            return (abs_error < k_) ? 0.0 : 0.5*(k_-abs_error)*(k_-abs_error);
          }
          double weight(double error) const {
            // note that this code is slightly uglier than above, because there are three distinct
            // cases to handle (left of deadzone, deadzone, right of deadzone) instead of the two
            // cases (deadzone, non-deadzone) above.
            if (fabs(error) <= k_) return 0.0;
            else if (error > k_) return (-k_+error)/error;
            else return (k_+error)/error;
          }
          void print(const std::string &s) const;
          bool equals(const Base& expected, double tol=1e-8) const;
          static shared_ptr Create(double k, const ReweightScheme reweight = Block);

      private:
          /** Serialization function */
          friend class boost::serialization::access;
          template<class ARCHIVE>
          void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
            ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
            ar & BOOST_SERIALIZATION_NVP(k_);
          }
      };

    } ///\namespace mEstimator

    /**
     *  Base class for robust error models
     *  The robust M-estimators above simply tell us how to re-weight the residual, and are
     *  isotropic kernels, in that they do not allow for correlated noise. They also have no way
     *  to scale the residual values, e.g., dividing by a single standard deviation.
     *  Hence, the actual robust noise model below does this scaling/whitening in sequence, by
     *  passing both a standard noise model and a robust estimator.
     *
     *  Taking as an example noise = Isotropic::Create(d, sigma),  we first divide the residuals
     *  uw = |Ax-b| by sigma by "whitening" the system (A,b), obtaining r = |Ax-b|/sigma, and
     *  then we pass the now whitened residual 'r' through the robust M-estimator.
     *  This is currently done by multiplying with sqrt(w), because the residuals will be squared
     *  again in error, yielding 0.5 \sum w(r)*r^2.
     *
     *  In other words, while sigma is expressed in the native residual units, a parameter like
     *  k in the Huber norm is expressed in whitened units, i.e., "nr of sigmas".
     */
    class GTSAM_EXPORT Robust : public Base {
    public:
      typedef boost::shared_ptr<Robust> shared_ptr;

    protected:
      typedef mEstimator::Base RobustModel;
      typedef noiseModel::Base NoiseModel;

      const RobustModel::shared_ptr robust_; ///< robust error function used
      const NoiseModel::shared_ptr noise_;   ///< noise model used

    public:

      /// Default Constructor for serialization
      Robust() {};

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

      // TODO: functions below are dummy but necessary for the noiseModel::Base
      inline virtual Vector whiten(const Vector& v) const
      { Vector r = v; this->WhitenSystem(r); return r; }
      inline virtual Matrix Whiten(const Matrix& A) const
      { Vector b; Matrix B=A; this->WhitenSystem(B,b); return B; }
      inline virtual Vector unwhiten(const Vector& /*v*/) const
      { throw std::invalid_argument("unwhiten is not currently supported for robust noise models."); }
      inline virtual double distance(const Vector& v) const
      { return this->whiten(v).squaredNorm(); }
      // TODO(mike): fold the use of the m-estimator residual(...) function into distance(...)
      inline virtual double distance_non_whitened(const Vector& v) const
      { return robust_->residual(v.norm()); }
      // TODO: these are really robust iterated re-weighting support functions
      virtual void WhitenSystem(Vector& b) const;
      virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const;
      virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

      static shared_ptr Create(
        const RobustModel::shared_ptr &robust, const NoiseModel::shared_ptr noise);

    private:
      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar & boost::serialization::make_nvp("robust_", const_cast<RobustModel::shared_ptr&>(robust_));
        ar & boost::serialization::make_nvp("noise_", const_cast<NoiseModel::shared_ptr&>(noise_));
      }
    };

    // Helper function
    GTSAM_EXPORT boost::optional<Vector> checkIfDiagonal(const Matrix M);

  } // namespace noiseModel

  /** Note, deliberately not in noiseModel namespace.
   * Deprecated. Only for compatibility with previous version.
   */
  typedef noiseModel::Base::shared_ptr SharedNoiseModel;
  typedef noiseModel::Gaussian::shared_ptr SharedGaussian;
  typedef noiseModel::Diagonal::shared_ptr SharedDiagonal;
  typedef noiseModel::Constrained::shared_ptr SharedConstrained;
  typedef noiseModel::Isotropic::shared_ptr SharedIsotropic;

  /// traits
  template<> struct traits<noiseModel::Gaussian> : public Testable<noiseModel::Gaussian> {};
  template<> struct traits<noiseModel::Diagonal> : public Testable<noiseModel::Diagonal> {};
  template<> struct traits<noiseModel::Constrained> : public Testable<noiseModel::Constrained> {};
  template<> struct traits<noiseModel::Isotropic> : public Testable<noiseModel::Isotropic> {};
  template<> struct traits<noiseModel::Unit> : public Testable<noiseModel::Unit> {};

} //\ namespace gtsam


