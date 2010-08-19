/**
 *  @file  PriorFactor.h
 *  @authors Frank Dellaert
 **/
#pragma once

#include <ostream>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam {

	/**
	 * A class for a soft prior on any Lie type
	 * It takes three template parameters:
	 *   T is the Lie group type for which the prior is define
	 *   Key (typically TypedSymbol) is used to look up T's in a Config
	 *   Config where the T's are stored, typically LieConfig<Key,T> or a TupleConfig<...>
	 * The Key type is not arbitrary: we need to cast to a Symbol at linearize, so
	 * a simple type like int will not work
	 */
	template<class Config, class Key, class T>
	class PriorFactor: public NonlinearFactor1<Config, Key, T> {

	private:

		typedef NonlinearFactor1<Config, Key, T> Base;

		T prior_; /** The measurement */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PriorFactor> shared_ptr;

		/** Constructor */
		PriorFactor(const Key& key, const T& prior,
				const SharedGaussian& model) :
			Base(model, key), prior_(prior) {
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			prior_.print("prior");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const PriorFactor<Config, Key, T> *e = dynamic_cast<const PriorFactor<
					Config, Key, T>*> (&expected);
			return e != NULL && Base::equals(expected) && this->prior_.equals(
					e->prior_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = eye(dim(p));
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return logmap(prior_, p);
		}
	};

} /// namespace gtsam
