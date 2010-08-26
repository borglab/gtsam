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
	template<class Config, class Key>
	class BetweenConstraint : public NonlinearEqualityConstraint2<Config, Key, Key> {
	public:
		typedef typename Key::Value_t X;

	protected:
		typedef NonlinearEqualityConstraint2<Config, Key, Key> Base;

		X measured_; /// fixed between value

	public:

		typedef boost::shared_ptr<BetweenConstraint<Config, Key> > shared_ptr;

		BetweenConstraint(const X& measured, const Key& key1, const Key& key2, double mu = 1000.0)
			: Base(key1, key2, X::Dim(), mu), measured_(measured) {}

		/** g(x) with optional derivative2 */
		Vector evaluateError(const X& x1, const X& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {
			X hx = x1.between(x2, H1, H2);
			return measured_.logmap(hx);
		}

		inline const X measured() const {
			return measured_;
		}
	};

} // \namespace gtsam
