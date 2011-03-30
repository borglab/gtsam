/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.h
 * @brief   Non-linear factor class
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <list>
#include <limits>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>

#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/SharedGaussian.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <gtsam/nonlinear/Ordering.h>

// FIXME: is this necessary? These don't even fit the right format
#define INSTANTIATE_NONLINEAR_FACTOR1(C,J) \
  template class gtsam::NonlinearFactor1<C,J>;
#define INSTANTIATE_NONLINEAR_FACTOR2(C,J1,J2) \
    template class gtsam::NonlinearFactor2<C,J1,J2>;
#define INSTANTIATE_NONLINEAR_FACTOR3(C,J1,J2,J3) \
    template class gtsam::NonlinearFactor3<C,J1,J2,J3>;

namespace gtsam {

	/**
	 * Nonlinear factor which assumes zero-mean Gaussian noise on the
	 * on a measurement predicted by a non-linear function h.
	 *
	 * Templated on a values structure type. The values structures are typically
	 * more general than just vectors, e.g., Rot3 or Pose3,
	 * which are objects in non-linear manifolds (Lie groups).
	 */
	template<class VALUES>
	class NonlinearFactor: public Factor<Symbol>, public Testable<NonlinearFactor<VALUES> > {

	protected:

		typedef NonlinearFactor<VALUES> This;

		SharedGaussian noiseModel_; /** Noise model */

	public:

		typedef boost::shared_ptr<NonlinearFactor<VALUES> > shared_ptr;

		/** Default constructor for I/O only */
		NonlinearFactor() {
		}

//		virtual ~NonlinearFactor() {}

		/**
		 *  Constructor
		 *  @param noiseModel shared pointer to a noise model
		 */
		NonlinearFactor(const SharedGaussian& noiseModel) :
			noiseModel_(noiseModel) {
		}

		/**
		 *  Constructor
		 *  @param z measurement
		 *  @param key by which to look up X value in Values
		 */
		NonlinearFactor(const SharedGaussian& noiseModel, const Symbol& key1) :
			Factor<Symbol>(key1), noiseModel_(noiseModel) {
		}

		/**
		 * Constructor
		 * @param j1 key of the first variable
		 * @param j2 key of the second variable
		 */
		NonlinearFactor(const SharedGaussian& noiseModel, const Symbol& j1, const Symbol& j2) :
			Factor<Symbol>(j1,j2), noiseModel_(noiseModel) {
		}

		/**
		 * Constructor - arbitrary number of keys
		 * @param keys is the set of Symbols in the factor
		 */
		NonlinearFactor(const SharedGaussian& noiseModel, const std::set<Symbol>& keys) :
			Factor<Symbol>(keys), noiseModel_(noiseModel) {
		}

		/** print */
		virtual void print(const std::string& s = "") const {
			std::cout << s << ": NonlinearFactor\n";
			noiseModel_->print("  noise model");
		}

		/** Check if two NonlinearFactor objects are equal */
		virtual bool equals(const NonlinearFactor<VALUES>& f, double tol = 1e-9) const {
			return noiseModel_->equals(*f.noiseModel_, tol);
		}

		/**
		 * calculate the error of the factor
		 * Override for systems with unusual noise models
		 */
		virtual double error(const VALUES& c) const {
			return 0.5 * noiseModel_->Mahalanobis(unwhitenedError(c));
		}

		/** get the dimension of the factor (number of rows on linearization) */
		size_t dim() const {
			return noiseModel_->dim();
		}

		/** access to the noise model */
		SharedGaussian get_noiseModel() const {
			return noiseModel_;
		}

		/** Vector of errors, unwhitened ! */
		virtual Vector unwhitenedError(const VALUES& c) const = 0;

		/** Vector of errors, whitened ! */
		Vector whitenedError(const VALUES& c) const {
			return noiseModel_->whiten(unwhitenedError(c));
		}

		/** linearize to a GaussianFactor */
		virtual boost::shared_ptr<JacobianFactor>
		linearize(const VALUES& c, const Ordering& ordering) const = 0;

		/**
		 * Create a symbolic factor using the given ordering to determine the
		 * variable indices.
		 */
		virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("Factor",
					boost::serialization::base_object<Factor<Symbol> >(*this));
			ar & BOOST_SERIALIZATION_NVP(noiseModel_);
		}

	}; // NonlinearFactor


	/**
	 * A Gaussian nonlinear factor that takes 1 parameter
	 * implementing the density P(z|x) \propto exp -0.5*|z-h(x)|^2_C
	 * Templated on the parameter type X and the values structure Values
	 * There is no return type specified for h(x). Instead, we require
	 * the derived class implements error_vector(c) = h(x)-z \approx Ax-b
	 * This allows a graph to have factors with measurements of mixed type.
	 */
	template<class VALUES, class KEY>
	class NonlinearFactor1: public NonlinearFactor<VALUES> {

	public:

		// typedefs for value types pulled from keys
		typedef typename KEY::Value X;

	protected:

		// The value of the key. Not const to allow serialization
		KEY key_;

		typedef NonlinearFactor<VALUES> Base;
		typedef NonlinearFactor1<VALUES, KEY> This;

	public:

		/** Default constructor for I/O only */
		NonlinearFactor1() {
		}

		virtual ~NonlinearFactor1() {}

		inline const KEY& key() const {
			return key_;
		}

		/**
		 *  Constructor
		 *  @param z measurement
		 *  @param key by which to look up X value in Values
		 */
		NonlinearFactor1(const SharedGaussian& noiseModel, const KEY& key1) :
			Base(noiseModel,key1), key_(key1) {
		}

		/* print */
		virtual void print(const std::string& s = "") const {
			std::cout << s << ": NonlinearFactor1\n";
			std::cout << "  key: " << (std::string) key_ << std::endl;
			this->noiseModel_->print("  noise model: ");
		}

		/** Check if two factors are equal. Note type is IndexFactor and needs cast. */
		virtual bool equals(const NonlinearFactor1<VALUES,KEY>& f, double tol = 1e-9) const {
			return Base::noiseModel_->equals(*f.noiseModel_, tol) && (key_ == f.key_);
		}

		/** error function h(x)-z, unwhitened !!! */
		inline Vector unwhitenedError(const VALUES& x) const {
			const KEY& j = key_;
			const X& xj = x[j];
			return evaluateError(xj);
		}

		/**
		 * Linearize a non-linearFactor1 to get a GaussianFactor
		 * Ax-b \approx h(x0+dx)-z = h(x0) + A*dx - z
		 * Hence b = z - h(x0) = - error_vector(x)
		 */
		virtual boost::shared_ptr<JacobianFactor> linearize(const VALUES& x, const Ordering& ordering) const {
			const X& xj = x[key_];
			Matrix A;
			Vector b = - evaluateError(xj, A);
			Index var = ordering[key_];
			// TODO pass unwhitened + noise model to Gaussian factor
			SharedDiagonal constrained =
					boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
			if (constrained.get() != NULL)
				return JacobianFactor::shared_ptr(new JacobianFactor(var, A, b, constrained));
			this->noiseModel_->WhitenInPlace(A);
			this->noiseModel_->whitenInPlace(b);
			return JacobianFactor::shared_ptr(new JacobianFactor(var, A, b,
					noiseModel::Unit::Create(b.size())));
		}

    /**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
    virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
      return IndexFactor::shared_ptr(new IndexFactor(ordering[key_]));
    }

		/*
		 *  Override this method to finish implementing a unary factor.
		 *  If the optional Matrix reference argument is specified, it should compute
		 *  both the function evaluation and its derivative in X.
		 */
		virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H =
				boost::none) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(key_);
		}

	};

	/**
	 * A Gaussian nonlinear factor that takes 2 parameters
	 */
	template<class VALUES, class KEY1, class KEY2>
	class NonlinearFactor2: public NonlinearFactor<VALUES> {

	  public:

		// typedefs for value types pulled from keys
		typedef typename KEY1::Value X1;
		typedef typename KEY2::Value X2;

	protected:

		// The values of the keys. Not const to allow serialization
		KEY1 key1_;
		KEY2 key2_;

		typedef NonlinearFactor<VALUES> Base;
		typedef NonlinearFactor2<VALUES, KEY1, KEY2> This;

	public:

		/**
		 * Default Constructor for I/O
		 */
		NonlinearFactor2() {
		}

		/**
		 * Constructor
		 * @param j1 key of the first variable
		 * @param j2 key of the second variable
		 */
		NonlinearFactor2(const SharedGaussian& noiseModel, KEY1 j1, KEY2 j2) :
			Base(noiseModel,j1,j2), key1_(j1), key2_(j2) {
		}

		virtual ~NonlinearFactor2() {}

		/** Print */
		virtual void print(const std::string& s = "") const {
			std::cout << s << ": NonlinearFactor2\n";
			std::cout << "  key1: " << (std::string) key1_ << "\n";
			std::cout << "  key2: " << (std::string) key2_ << "\n";
			this->noiseModel_->print("  noise model: ");
		}

		/** Check if two factors are equal */
		virtual bool equals(const NonlinearFactor2<VALUES,KEY1,KEY2>& f, double tol = 1e-9) const {
			return Base::noiseModel_->equals(*f.noiseModel_, tol) && (key1_ == f.key1_)
					&& (key2_ == f.key2_);
		}

		/** error function z-h(x1,x2) */
		inline Vector unwhitenedError(const VALUES& x) const {
			const X1& x1 = x[key1_];
			const X2& x2 = x[key2_];
			return evaluateError(x1, x2);
		}

		/**
		 * Linearize a non-linearFactor2 to get a GaussianFactor
		 * Ax-b \approx h(x1+dx1,x2+dx2)-z = h(x1,x2) + A2*dx1 + A2*dx2 - z
		 * Hence b = z - h(x1,x2) = - error_vector(x)
		 */
		boost::shared_ptr<JacobianFactor> linearize(const VALUES& c, const Ordering& ordering) const {
			const X1& x1 = c[key1_];
			const X2& x2 = c[key2_];
			Matrix A1, A2;
			Vector b = -evaluateError(x1, x2, A1, A2);
			const Index var1 = ordering[key1_], var2 = ordering[key2_];
			// TODO pass unwhitened + noise model to Gaussian factor
			SharedDiagonal constrained =
					boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
			if (constrained.get() != NULL) {
				return JacobianFactor::shared_ptr(new JacobianFactor(var1, A1, var2,
						A2, b, constrained));
			}
			this->noiseModel_->WhitenInPlace(A1);
			this->noiseModel_->WhitenInPlace(A2);
			this->noiseModel_->whitenInPlace(b);
			if(var1 < var2)
			  return JacobianFactor::shared_ptr(new JacobianFactor(var1, A1, var2,
			      A2, b, noiseModel::Unit::Create(b.size())));
			else
			  return JacobianFactor::shared_ptr(new JacobianFactor(var2, A2, var1,
			      A1, b, noiseModel::Unit::Create(b.size())));
		}

		/**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
    virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
      const Index var1 = ordering[key1_], var2 = ordering[key2_];
      if(var1 < var2)
        return IndexFactor::shared_ptr(new IndexFactor(var1, var2));
      else
        return IndexFactor::shared_ptr(new IndexFactor(var2, var1));
    }

		/** methods to retrieve both keys */
		inline const KEY1& key1() const {
			return key1_;
		}
		inline const KEY2& key2() const {
			return key2_;
		}

		/*
		 *  Override this method to finish implementing a binary factor.
		 *  If any of the optional Matrix reference arguments are specified, it should compute
		 *  both the function evaluation and its derivative(s) in X1 (and/or X2).
		 */
		virtual Vector
		evaluateError(const X1&, const X2&, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const = 0;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(key1_);
			ar & BOOST_SERIALIZATION_NVP(key2_);
		}

	};

/* ************************************************************************* */

  /**
   * A Gaussian nonlinear factor that takes 3 parameters
   */
  template<class VALUES, class KEY1, class KEY2, class KEY3>
  class NonlinearFactor3: public NonlinearFactor<VALUES> {

  public:

	// typedefs for value types pulled from keys
	typedef typename KEY1::Value X1;
	typedef typename KEY2::Value X2;
	typedef typename KEY3::Value X3;

  protected:

	// The values of the keys. Not const to allow serialization
    KEY1 key1_;
    KEY2 key2_;
    KEY3 key3_;

    typedef NonlinearFactor<VALUES> Base;
    typedef NonlinearFactor3<VALUES, KEY1, KEY2, KEY3> This;

  public:

    /**
     * Default Constructor for I/O
     */
    NonlinearFactor3() {
    }

    /**
     * Constructor
     * @param j1 key of the first variable
     * @param j2 key of the second variable
     * @param j3 key of the third variable
     */
    NonlinearFactor3(const SharedGaussian& noiseModel, KEY1 j1, KEY2 j2, KEY3 j3) :
      Base(noiseModel), key1_(j1), key2_(j2), key3_(j3) {
      this->keys_.reserve(3);
      this->keys_.push_back(key1_);
      this->keys_.push_back(key2_);
      this->keys_.push_back(key3_);
    }

    virtual ~NonlinearFactor3() {}

    /** Print */
    virtual void print(const std::string& s = "") const {
      std::cout << s << ": NonlinearFactor3\n";
      std::cout << "  key1: " << (std::string) key1_ << "\n";
      std::cout << "  key2: " << (std::string) key2_ << "\n";
      std::cout << "  key3: " << (std::string) key3_ << "\n";
      this->noiseModel_->print("  noise model: ");
    }

    /** Check if two factors are equal */
    virtual bool equals(const NonlinearFactor3<VALUES,KEY1,KEY2,KEY3>& f, double tol = 1e-9) const {
      return Base::noiseModel_->equals(*f.noiseModel_, tol) && (key1_ == f.key1_)
          && (key2_ == f.key2_) && (key3_ == f.key3_);
    }

    /** error function z-h(x1,x2) */
    inline Vector unwhitenedError(const VALUES& x) const {
      const X1& x1 = x[key1_];
      const X2& x2 = x[key2_];
      const X3& x3 = x[key3_];
      return evaluateError(x1, x2, x3);
    }

    /**
     * Linearize a non-linearFactor2 to get a GaussianFactor
     * Ax-b \approx h(x1+dx1,x2+dx2,x3+dx3)-z = h(x1,x2,x3) + A2*dx1 + A2*dx2 + A3*dx3 - z
     * Hence b = z - h(x1,x2,x3) = - error_vector(x)
     */
    boost::shared_ptr<JacobianFactor> linearize(const VALUES& c, const Ordering& ordering) const {
      const X1& x1 = c[key1_];
      const X2& x2 = c[key2_];
      const X3& x3 = c[key3_];
      Matrix A1, A2, A3;
      Vector b = -evaluateError(x1, x2, x3, A1, A2, A3);
      const Index var1 = ordering[key1_], var2 = ordering[key2_], var3 = ordering[key3_];
      // TODO pass unwhitened + noise model to Gaussian factor
      SharedDiagonal constrained =
          boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
      if (constrained.get() != NULL) {
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var1, A1, var2, A2, var3, A3, b, constrained));
      }
      this->noiseModel_->WhitenInPlace(A1);
      this->noiseModel_->WhitenInPlace(A2);
      this->noiseModel_->WhitenInPlace(A3);
      this->noiseModel_->whitenInPlace(b);
      if(var1 < var2 && var2 < var3)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var1, A1, var2, A2, var3, A3, b, noiseModel::Unit::Create(b.size())));
      else if(var2 < var1 && var1 < var3)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var2, A2, var1, A1, var3, A3, b, noiseModel::Unit::Create(b.size())));
      else if(var1 < var3 && var3 < var2)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var1, A1, var3, A3, var2, A2, b, noiseModel::Unit::Create(b.size())));
      else if(var2 < var3 && var3 < var1)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var2, A2, var3, A3, var1, A1, b, noiseModel::Unit::Create(b.size())));
      else if(var3 < var1 && var1 < var2)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var3, A3, var1, A1, var2, A2, b, noiseModel::Unit::Create(b.size())));
      else if(var3 < var2 && var2 < var1)
        return JacobianFactor::shared_ptr(
            new JacobianFactor(var3, A3, var2, A2, var1, A1, b, noiseModel::Unit::Create(b.size())));
      else {
        assert(false);
        return JacobianFactor::shared_ptr();
      }
    }

    /**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
    virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
      const Index var1 = ordering[key1_], var2 = ordering[key2_], var3 = ordering[key3_];
      if(var1 < var2 && var2 < var3)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key1_], ordering[key2_], ordering[key3_]));
      else if(var2 < var1 && var1 < var3)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key2_], ordering[key1_], ordering[key3_]));
      else if(var1 < var3 && var3 < var2)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key1_], ordering[key3_], ordering[key2_]));
      else if(var2 < var3 && var3 < var1)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key2_], ordering[key3_], ordering[key1_]));
      else if(var3 < var1 && var1 < var2)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key3_], ordering[key1_], ordering[key2_]));
      else if(var3 < var2 && var2 < var1)
        return IndexFactor::shared_ptr(new IndexFactor(ordering[key3_], ordering[key2_], ordering[key1_]));
      else {
        assert(false);
        return GaussianFactor::shared_ptr();
      }
    }

    /** methods to retrieve keys */
    inline const KEY1& key1() const {
      return key1_;
    }
    inline const KEY2& key2() const {
      return key2_;
    }
    inline const KEY3& key3() const {
      return key3_;
    }

    /*
     *  Override this method to finish implementing a trinary factor.
     *  If any of the optional Matrix reference arguments are specified, it should compute
     *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
     */
    virtual Vector
    evaluateError(const X1&, const X2&, const X3&,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none) const = 0;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NonlinearFactor",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(key1_);
      ar & BOOST_SERIALIZATION_NVP(key2_);
      ar & BOOST_SERIALIZATION_NVP(key3_);
    }

  };

/* ************************************************************************* */

}
