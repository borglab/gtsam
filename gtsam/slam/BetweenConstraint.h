/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BetweenConstraint.h
 * @brief Implements a constraint to force a between
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearConstraint.h>

namespace gtsam {

	/**
	 * Binary between constraint - forces between to a given value
	 * This constraint requires the underlying type to a Lie type
	 */
	template<class VALUES, class KEY>
	class BetweenConstraint : public NonlinearEqualityConstraint2<VALUES, KEY, KEY> {
	public:
		typedef typename KEY::Value X;

	protected:
		typedef NonlinearEqualityConstraint2<VALUES, KEY, KEY> Base;

		X measured_; /// fixed between value

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(X)

	public:

		typedef boost::shared_ptr<BetweenConstraint<VALUES, KEY> > shared_ptr;

		BetweenConstraint(const X& measured, const KEY& key1, const KEY& key2, double mu = 1000.0)
			: Base(key1, key2, X::Dim(), mu), measured_(measured) {}

		/** g(x) with optional derivative2 */
		Vector evaluateError(const X& x1, const X& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {
			X hx = x1.between(x2, H1, H2);
			return measured_.unretract(hx);
		}

		inline const X measured() const {
			return measured_;
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearEqualityConstraint2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	};

} // \namespace gtsam
