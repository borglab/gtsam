/**
 * @file BoundingConstraint.h
 * @brief Provides partially implemented constraints to implement bounds
 * @author Alex Cunningham
 */

#pragma once

#include <Lie.h>
#include <NonlinearConstraint.h>

namespace gtsam {

	/**
	 * Unary inequality constraint forcing a scalar to be
	 * greater/less than a fixed threshold.  The function
	 * will need to have its value function implemented to return
	 * a scalar for comparison.
	 */
	template<class Cfg, class Key, class X>
	struct BoundingConstraint1: public NonlinearConstraint1<Cfg, Key, X> {
		typedef NonlinearConstraint1<Cfg, Key, X> Base;
		typedef boost::shared_ptr<BoundingConstraint1<Cfg, Key, X> > shared_ptr;

		double threshold_;
		bool isGreaterThan_; /// flag for greater/less than

		BoundingConstraint1(const Key& key, double threshold,
				bool isGreaterThan, double mu = 1000.0) :
				Base(key, 1, mu), threshold_(threshold), isGreaterThan_(isGreaterThan) {
		}

		inline double threshold() const { return threshold_; }
		inline bool isGreaterThan() const { return isGreaterThan_; }

		/**
		 * function producing a scalar value to compare to the threshold
		 * Must have optional argument for derivative with 1xN matrix, where
		 * N = X::dim()
		 */
		virtual double value(const X& x, boost::optional<Matrix&> H =
				boost::none) const = 0;

		/** active when constraint *NOT* met */
		bool active(const Cfg& c) const {
			// note: still active at equality to avoid zigzagging
			double x = value(c[this->key_]);
			return (isGreaterThan_) ? x <= threshold_ : x >= threshold_;
		}

		Vector evaluateError(const X& x, boost::optional<Matrix&> H =
				boost::none) const {
			Matrix D;
			double error = value(x, D) - threshold_;
			if (H) *H = (isGreaterThan_) ? D : -1.0 * D;
			return (isGreaterThan_) ? Vector_(1, error) : -1.0 * Vector_(1, error);
		}
	};

	/**
	 * Binary scalar inequality constraint, with a similar value() function
	 * to implement for specific systems
	 */
	template<class Cfg, class Key1, class X1, class Key2, class X2>
	struct BoundingConstraint2: public NonlinearConstraint2<Cfg, Key1, X1, Key2, X2> {
		typedef NonlinearConstraint2<Cfg, Key1, X1, Key2, X2> Base;
		typedef boost::shared_ptr<BoundingConstraint2<Cfg, Key1, X1, Key2, X2> > shared_ptr;

		double threshold_;
		bool isGreaterThan_; /// flag for greater/less than

		BoundingConstraint2(const Key1& key1, const Key2& key2, double threshold,
				bool isGreaterThan, double mu = 1000.0)
			: Base(key1, key2, 1, mu), threshold_(threshold), isGreaterThan_(isGreaterThan) {}

		inline double threshold() const { return threshold_; }
		inline bool isGreaterThan() const { return isGreaterThan_; }

		/**
		 * function producing a scalar value to compare to the threshold
		 * Must have optional argument for derivatives)
		 */
		virtual double value(const X1& x1, const X2& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const = 0;

		/** active when constraint *NOT* met */
		bool active(const Cfg& c) const {
			// note: still active at equality to avoid zigzagging
			double x = value(c[this->key1_], c[this->key2_]);
			return (isGreaterThan_) ? x <= threshold_ : x >= threshold_;
		}

		Vector evaluateError(const X1& x1, const X2& x2,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const {
			Matrix D1, D2;
			double error = value(x1, x2, D1, D2) - threshold_;
			if (H1) *H1 = (isGreaterThan_) ? D1 : -1.0 * D1;
			if (H2) *H2 = (isGreaterThan_) ? D2 : -1.0 * D2;
			return (isGreaterThan_) ? Vector_(1, error) : -1.0 * Vector_(1, error);
		}
	};

} // \namespace gtsam
