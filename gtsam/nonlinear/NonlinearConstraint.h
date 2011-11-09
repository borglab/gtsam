/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file NonlinearConstraint.h
 * @brief Implements simple cases of constraints
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

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
};

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
};

}
