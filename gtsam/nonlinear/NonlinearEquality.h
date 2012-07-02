/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file NonlinearEquality.h
 * @brief Factor to handle enforced equality between factors
 * @author Alex Cunningham
 */

#pragma once

#include <limits>
#include <iostream>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

	/**
	 * Template default compare function that assumes a testable T
	 */
	template<class T>
	bool compare(const T& a, const T& b) {
		GTSAM_CONCEPT_TESTABLE_TYPE(T);
		return a.equals(b);
	}

	/**
	 * An equality factor that forces either one variable to a constant,
	 * or a set of variables to be equal to each other.
	 *
	 * Depending on flag, throws an error at linearization if the constraints are not met.
	 *
	 * Switchable implementation:
	 *   - ALLLOW_ERROR : if we allow that there can be nonzero error, does not throw, and uses gain
	 *   - ONLY_EXACT   : throws error at linearization if not at exact feasible point, and infinite error
	 *
	 * \nosubgrouping
	 */
	template<class VALUE>
	class NonlinearEquality: public NoiseModelFactor1<VALUE> {

	public:
		typedef VALUE T;

	private:

		// feasible value
		T feasible_;

		// error handling flag
		bool allow_error_;

		// error gain in allow error case
		double error_gain_;

		// typedef to this class
		typedef NonlinearEquality<VALUE> This;

		// typedef to base class
		typedef NoiseModelFactor1<VALUE> Base;

	public:

		/**
		 * Function that compares two values
		 */
		bool (*compare_)(const T& a, const T& b);


		/** default constructor - only for serialization */
		NonlinearEquality() {}

		virtual ~NonlinearEquality() {}

		/// @name Standard Constructors
		/// @{

		/**
		 * Constructor - forces exact evaluation
		 */
		NonlinearEquality(Key j, const T& feasible, bool (*_compare)(const T&, const T&) = compare<T>) :
			Base(noiseModel::Constrained::All(feasible.dim()), j), feasible_(feasible),
			allow_error_(false), error_gain_(0.0),
			compare_(_compare) {
		}

		/**
		 * Constructor - allows inexact evaluation
		 */
		NonlinearEquality(Key j, const T& feasible, double error_gain, bool (*_compare)(const T&, const T&) = compare<T>) :
			Base(noiseModel::Constrained::All(feasible.dim()), j), feasible_(feasible),
			allow_error_(true), error_gain_(error_gain),
			compare_(_compare) {
		}

		/// @}
		/// @name Testable
		/// @{

		virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			std::cout << s << "Constraint: on [" << keyFormatter(this->key()) << "]\n";
			gtsam::print(feasible_,"Feasible Point:\n");
			std::cout << "Variable Dimension: " << feasible_.dim() << std::endl;
		}

		/** Check if two factors are equal */
		virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
		  const This* e = dynamic_cast<const This*>(&f);
		  return e && Base::equals(f) && feasible_.equals(e->feasible_, tol) &&
          fabs(error_gain_ - e->error_gain_) < tol;
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** actual error function calculation */
		virtual double error(const Values& c) const {
			const T& xj = c.at<T>(this->key());
			Vector e = this->unwhitenedError(c);
			if (allow_error_ || !compare_(xj, feasible_)) {
				return error_gain_ * dot(e,e);
			} else {
				return 0.0;
			}
		}

		/** error function */
		Vector evaluateError(const T& xj, boost::optional<Matrix&> H = boost::none) const {
			size_t nj = feasible_.dim();
			if (allow_error_) {
				if (H) *H = eye(nj); // FIXME: this is not the right linearization for nonlinear compare
				return xj.localCoordinates(feasible_);
			} else if (compare_(feasible_,xj)) {
				if (H) *H = eye(nj);
				return zero(nj); // set error to zero if equal
			} else {
				if (H) throw std::invalid_argument(
						"Linearization point not feasible for " + DefaultKeyFormatter(this->key()) + "!");
				return repeat(nj, std::numeric_limits<double>::infinity()); // set error to infinity if not equal
			}
		}

		// Linearize is over-written, because base linearization tries to whiten
		virtual GaussianFactor::shared_ptr linearize(const Values& x, const Ordering& ordering) const {
			const T& xj = x.at<T>(this->key());
			Matrix A;
			Vector b = evaluateError(xj, A);
			SharedDiagonal model = noiseModel::Constrained::All(b.size());
			return GaussianFactor::shared_ptr(new JacobianFactor(ordering[this->key()], A, b, model));
		}

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/// @}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor1",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(feasible_);
			ar & BOOST_SERIALIZATION_NVP(allow_error_);
			ar & BOOST_SERIALIZATION_NVP(error_gain_);
		}

	}; // \class NonlinearEquality

	/* ************************************************************************* */
	/**
	 * Simple unary equality constraint - fixes a value for a variable
	 */
	template<class VALUE>
	class NonlinearEquality1 : public NoiseModelFactor1<VALUE> {

	public:
		typedef VALUE X;

	protected:
		typedef NoiseModelFactor1<VALUE> Base;
		typedef NonlinearEquality1<VALUE> This;

		/** default constructor to allow for serialization */
		NonlinearEquality1() {}

		X value_; /// fixed value for variable

		GTSAM_CONCEPT_MANIFOLD_TYPE(X);
		GTSAM_CONCEPT_TESTABLE_TYPE(X);

	public:

		typedef boost::shared_ptr<NonlinearEquality1<VALUE> > shared_ptr;

		///TODO: comment
		NonlinearEquality1(const X& value, Key key1, double mu = 1000.0)
			: Base(noiseModel::Constrained::All(value.dim(), fabs(mu)), key1), value_(value) {}

		virtual ~NonlinearEquality1() {}

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/** g(x) with optional derivative */
		Vector evaluateError(const X& x1, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = eye(x1.dim());
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return value_.localCoordinates(x1);
		}

		/** Print */
	  virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
	    std::cout << s << ": NonlinearEquality1("
	    		<< keyFormatter(this->key()) << "),"<< "\n";
	    this->noiseModel_->print();
	    value_.print("Value");
	  }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor1",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(value_);
		}
	}; // \NonlinearEquality1

	/* ************************************************************************* */
	/**
	 * Simple binary equality constraint - this constraint forces two factors to
	 * be the same.
	 */
	template<class VALUE>
	class NonlinearEquality2 : public NoiseModelFactor2<VALUE, VALUE> {
	public:
		typedef VALUE X;

	protected:
		typedef NoiseModelFactor2<VALUE, VALUE> Base;
		typedef NonlinearEquality2<VALUE> This;

		GTSAM_CONCEPT_MANIFOLD_TYPE(X);

		/** default constructor to allow for serialization */
		NonlinearEquality2() {}

	public:

		typedef boost::shared_ptr<NonlinearEquality2<VALUE> > shared_ptr;

		///TODO: comment
		NonlinearEquality2(Key key1, Key key2, double mu = 1000.0)
			: Base(noiseModel::Constrained::All(X::Dim(), fabs(mu)), key1, key2) {}
		virtual ~NonlinearEquality2() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/** g(x) with optional derivative2 */
		Vector evaluateError(const X& x1, const X& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {
			const size_t p = X::Dim();
			if (H1) *H1 = -eye(p);
			if (H2) *H2 = eye(p);
			return x1.localCoordinates(x2);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
					boost::serialization::base_object<Base>(*this));
		}
	}; // \NonlinearEquality2

} // namespace gtsam

