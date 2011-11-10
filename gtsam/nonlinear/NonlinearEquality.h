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
	 */
	template<class VALUES, class KEY>
	class NonlinearEquality: public NonlinearFactor1<VALUES, KEY> {

	public:
		typedef typename KEY::Value T;

	private:

		// feasible value
		T feasible_;

		// error handling flag
		bool allow_error_;

		// error gain in allow error case
		double error_gain_;

	public:

		/**
		 * Function that compares two values
		 */
		bool (*compare_)(const T& a, const T& b);

		typedef NonlinearFactor1<VALUES, KEY> Base;

		/** default constructor - only for serialization */
		NonlinearEquality() {}

		virtual ~NonlinearEquality() {}

		/**
		 * Constructor - forces exact evaluation
		 */
		NonlinearEquality(const KEY& j, const T& feasible, bool (*_compare)(const T&, const T&) = compare<T>) :
			Base(noiseModel::Constrained::All(feasible.dim()), j), feasible_(feasible),
			allow_error_(false), error_gain_(0.0),
			compare_(_compare) {
		}

		/**
		 * Constructor - allows inexact evaluation
		 */
		NonlinearEquality(const KEY& j, const T& feasible, double error_gain, bool (*_compare)(const T&, const T&) = compare<T>) :
			Base(noiseModel::Constrained::All(feasible.dim()), j), feasible_(feasible),
			allow_error_(true), error_gain_(error_gain),
			compare_(_compare) {
		}

		void print(const std::string& s = "") const {
			std::cout << "Constraint: " << s << " on [" << (std::string)(this->key_) << "]\n";
			gtsam::print(feasible_,"Feasible Point");
			std::cout << "Variable Dimension: " << feasible_.dim() << std::endl;
		}

		/** Check if two factors are equal */
		bool equals(const NonlinearEquality<VALUES,KEY>& f, double tol = 1e-9) const {
			if (!Base::equals(f)) return false;
			return feasible_.equals(f.feasible_, tol) &&
					fabs(error_gain_ - f.error_gain_) < tol;
		}

		/** actual error function calculation */
		virtual double error(const VALUES& c) const {
			const T& xj = c[this->key_];
			Vector e = this->unwhitenedError(c);
			if (allow_error_ || !compare_(xj, feasible_)) {
				return error_gain_ * dot(e,e);
			} else {
				return 0.0;
			}
		}

		/** error function */
		inline Vector evaluateError(const T& xj, boost::optional<Matrix&> H = boost::none) const {
			size_t nj = feasible_.dim();
			if (allow_error_) {
				if (H) *H = eye(nj); // FIXME: this is not the right linearization for nonlinear compare
				return xj.localCoordinates(feasible_);
			} else if (compare_(feasible_,xj)) {
				if (H) *H = eye(nj);
				return zero(nj); // set error to zero if equal
			} else {
				if (H) throw std::invalid_argument(
						"Linearization point not feasible for " + (std::string)(this->key_) + "!");
				return repeat(nj, std::numeric_limits<double>::infinity()); // set error to infinity if not equal
			}
		}

		// Linearize is over-written, because base linearization tries to whiten
		virtual GaussianFactor::shared_ptr linearize(const VALUES& x, const Ordering& ordering) const {
			const T& xj = x[this->key_];
			Matrix A;
			Vector b = evaluateError(xj, A);
			SharedDiagonal model = noiseModel::Constrained::All(b.size());
			return GaussianFactor::shared_ptr(new JacobianFactor(ordering[this->key_], A, b, model));
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor1",
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
	template<class VALUES, class KEY>
	class NonlinearEquality1 : public NonlinearFactor1<VALUES, KEY> {

	public:
		typedef typename KEY::Value X;

	protected:
		typedef NonlinearFactor1<VALUES, KEY> Base;

		/** default constructor to allow for serialization */
		NonlinearEquality1() {}

		X value_; /// fixed value for variable

		GTSAM_CONCEPT_MANIFOLD_TYPE(X);
		GTSAM_CONCEPT_TESTABLE_TYPE(X);

	public:

		typedef boost::shared_ptr<NonlinearEquality1<VALUES, KEY> > shared_ptr;

		NonlinearEquality1(const X& value, const KEY& key1, double mu = 1000.0)
			: Base(noiseModel::Constrained::All(value.dim(), fabs(mu)), key1), value_(value) {}

		virtual ~NonlinearEquality1() {}

		/** g(x) with optional derivative */
		Vector evaluateError(const X& x1, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = eye(x1.dim());
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return value_.localCoordinates(x1);
		}

		/** Print */
	  virtual void print(const std::string& s = "") const {
	    std::cout << s << ": NonlinearEquality1("
	    		<< (std::string) this->key_ << "),"<< "\n";
	    this->noiseModel_->print();
	    value_.print("Value");
	  }

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor1",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(value_);
		}
	}; // \NonlinearEquality1

	/* ************************************************************************* */
	/**
	 * Simple binary equality constraint - this constraint forces two factors to
	 * be the same.
	 */
	template<class VALUES, class KEY>
	class NonlinearEquality2 : public NonlinearFactor2<VALUES, KEY, KEY> {
	public:
		typedef typename KEY::Value X;

	protected:
		typedef NonlinearFactor2<VALUES, KEY, KEY> Base;

		GTSAM_CONCEPT_MANIFOLD_TYPE(X);

		/** default constructor to allow for serialization */
		NonlinearEquality2() {}

	public:

		typedef boost::shared_ptr<NonlinearEquality2<VALUES, KEY> > shared_ptr;

		NonlinearEquality2(const KEY& key1, const KEY& key2, double mu = 1000.0)
			: Base(noiseModel::Constrained::All(X::Dim(), fabs(mu)), key1, key2) {}
		virtual ~NonlinearEquality2() {}

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
			ar & boost::serialization::make_nvp("NonlinearFactor2",
					boost::serialization::base_object<Base>(*this));
		}
	}; // \NonlinearEquality2

} // namespace gtsam

