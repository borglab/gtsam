#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/Matrix.h>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/optional.hpp>

namespace gtsam {
  namespace noiseModel {
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
        ReweightScheme reweightScheme() const { return reweight_; }

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
        double weight(double error) const {
          return sqrtWeight(error)*sqrtWeight(error);
        }

        virtual void print(const std::string &s) const = 0;
        virtual bool equals(const Base& expected, double tol=1e-8) const = 0;

        virtual double sqrtWeight(double error) const = 0;

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
        virtual double sqrtWeight(double /*error*/) const { return 1.0; }
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
        double sqrtWeight(double error) const {
          return 1.0 / sqrt(1.0 + fabs(error) / c_);
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
        double sqrtWeight(double error) const {
          return (error < k_) ? (1.0) : sqrt(k_ / fabs(error));
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
        double sqrtWeight(double error) const {
          return k_ / sqrt(ksquared_ + error*error);
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
        double sqrtWeight(double error) const {
          if (std::fabs(error) <= c_) {
            double xc2 = error*error/csquared_;
            return (1.0-xc2);
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
        double sqrtWeight(double error) const {
          double xc2 = (error*error)/csquared_;
          return std::exp(-xc2/2.0);
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
        virtual double sqrtWeight(double error) const;
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
        virtual double sqrtWeight(double error) const;
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
          double sqrtWeight(double error) const {
            // note that this code is slightly uglier than above, because there are three distinct
            // cases to handle (left of deadzone, deadzone, right of deadzone) instead of the two
            // cases (deadzone, non-deadzone) above.
            if (fabs(error) <= k_) return 0.0;
            else if (error > k_) return sqrt((-k_+error)/error);
            else return sqrt((k_+error)/error);
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
  }
}