/**
 *  @file  BetweenFactor.h
 *  @authors Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "Lie.h"
#include "Matrix.h"

namespace gtsam {

	/**
	 * A class for a measurement predicted by "between(config[key1],config[key2])"
	 * T is the Lie group type, Config where the T's are gotten from
	 */
	template<class Config, class Key1, class T, class Key2 = Key1>
	class BetweenFactor: public NonlinearFactor2<Config, Key1, T, Key2, T> {

	private:

		typedef NonlinearFactor2<Config, Key1, T, Key2, T> Base;

		T measured_; /** The measurement */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<BetweenFactor> shared_ptr;

		/** Constructor */
		BetweenFactor(const Key1& key1, const Key2& key2, const T& measured,
				const SharedGaussian& model) :
			Base(model, key1, key2), measured_(measured) {
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			measured_.print("measured");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const BetweenFactor<Config, Key1, T, Key2> *e =
					dynamic_cast<const BetweenFactor<Config, Key1, T>*> (&expected);
			return e != NULL && Base::equals(expected) && this->measured_.equals(
					e->measured_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p1, const T& p2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			T hx = between(p1, p2, H1, H2); // h(x)
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return logmap(measured_, hx);
		}

		/** return the measured */
		inline const T measured() const {
			return measured_;
		}

		/** number of variables attached to this factor */
		inline std::size_t size() const {
			return 2;
		}
	};

} /// namespace gtsam
