/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PriorFactor.h
 *  @author Frank Dellaert
 **/
#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * A class for a soft prior on any Lie type
	 * It takes two template parameters:
	 *   Key (typically TypedSymbol) is used to look up T's in a Values
	 *   Values where the T's are stored, typically LieValues<Key> or a TupleValues<...>
	 * The Key type is not arbitrary: we need to cast to a Symbol at linearize, so
	 * a simple type like int will not work
	 */
	template<class VALUES, class KEY>
	class PriorFactor: public NonlinearFactor1<VALUES, KEY> {

	public:
		typedef typename KEY::Value T;

	private:

		typedef NonlinearFactor1<VALUES, KEY> Base;

		T prior_; /** The measurement */

		/** concept check by type */
		GTSAM_CONCEPT_TESTABLE_TYPE(T)

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PriorFactor> shared_ptr;

		/** default constructor - only use for serialization */
		PriorFactor() {}

		virtual ~PriorFactor() {}

		/** Constructor */
		PriorFactor(const KEY& key, const T& prior,
				const SharedNoiseModel& model) :
			Base(model, key), prior_(prior) {
		}

		/** implement functions needed for Testable */

		/** print */
		virtual void print(const std::string& s) const {
			std::cout << s << ": PriorFactor(" << (std::string) this->key_ << ")\n";
			prior_.print("  prior");
			this->noiseModel_->print("  noise model");
		}

		/** equals */
		virtual bool equals(const NonlinearFactor<VALUES>& expected, double tol=1e-9) const {
			const PriorFactor<VALUES, KEY> *e = dynamic_cast<const PriorFactor<
					VALUES, KEY>*> (&expected);
			return e != NULL && Base::equals(*e, tol) && this->prior_.equals(e->prior_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = eye(p.dim());
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return prior_.logmap(p);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor1",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(prior_);
		}
	};

} /// namespace gtsam
