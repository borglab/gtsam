/**
 * @file BetweenConstraint.h
 * @brief Implements a constraint to force a between
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearConstraint.h>

namespace gtsam {

	/**
	 * Binary between constraint - forces between to a given value
	 * This constraint requires the underlying type to a Lie type
	 */
	template<class Config, class Key, class X>
	class BetweenConstraint : public NonlinearEqualityConstraint2<Config, Key, X, Key, X> {
	protected:
		typedef NonlinearEqualityConstraint2<Config, Key, X, Key, X> Base;

		X measured_; /// fixed between value

	public:

		typedef boost::shared_ptr<BetweenConstraint<Config, Key, X> > shared_ptr;

		BetweenConstraint(const X& measured, const Key& key1, const Key& key2, double mu = 1000.0)
			: Base(key1, key2, X::dim(), mu), measured_(measured) {}

		/** g(x) with optional derivative2 */
		Vector evaluateError(const X& x1, const X& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {
			X hx = between(x1, x2, H1, H2);
			return logmap(measured_, hx);
		}

		inline const X measured() const {
			return measured_;
		}
	};

} // \namespace gtsam
