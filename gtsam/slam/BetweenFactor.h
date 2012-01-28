/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenFactor.h
 *  @author Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

	/**
	 * A class for a measurement predicted by "between(config[key1],config[key2])"
	 * KEY1::Value is the Lie Group type
	 * T is the measurement type, by default the same
	 */
	template<class KEY1, class T = typename KEY1::Value>
	class BetweenFactor: public NonlinearFactor2<KEY1, KEY1> {

	private:

		typedef BetweenFactor<KEY1, T> This;
		typedef NonlinearFactor2<KEY1, KEY1> Base;

		T measured_; /** The measurement */

		/** concept check by type */
		GTSAM_CONCEPT_LIE_TYPE(T)
		GTSAM_CONCEPT_TESTABLE_TYPE(T)

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<BetweenFactor> shared_ptr;

		/** default constructor - only use for serialization */
		BetweenFactor() {}

		/** Constructor */
		BetweenFactor(const KEY1& key1, const KEY1& key2, const T& measured,
				const SharedNoiseModel& model) :
			Base(model, key1, key2), measured_(measured) {
		}

		virtual ~BetweenFactor() {}

		/** implement functions needed for Testable */

		/** print */
		virtual void print(const std::string& s) const {
	    std::cout << s << "BetweenFactor("
	    		<< (std::string) this->key1_ << ","
	    		<< (std::string) this->key2_ << ")\n";
			measured_.print("  measured");
	    this->noiseModel_->print("  noise model");
		}

		/** equals */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =	dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) && this->measured_.equals(e->measured_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const typename KEY1::Value& p1, const typename KEY1::Value& p2,
				boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
						boost::none) const {
			T hx = p1.between(p2, H1, H2); // h(x)
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return measured_.localCoordinates(hx);
		}

		/** return the measured */
		inline const T measured() const {
			return measured_;
		}

		/** number of variables attached to this factor */
		inline std::size_t size() const {
			return 2;
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor2",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	}; // \class BetweenFactor

	/**
	 * Binary between constraint - forces between to a given value
	 * This constraint requires the underlying type to a Lie type
	 *
	 */
	template<class VALUES, class KEY>
	class BetweenConstraint : public BetweenFactor<VALUES, KEY> {
	public:
		typedef boost::shared_ptr<BetweenConstraint<VALUES, KEY> > shared_ptr;

		/** Syntactic sugar for constrained version */
		BetweenConstraint(const typename KEY::Value& measured,
				const KEY& key1, const KEY& key2, double mu = 1000.0)
			: BetweenFactor<VALUES, KEY>(key1, key2, measured,
					noiseModel::Constrained::All(KEY::Value::Dim(), fabs(mu))) {}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("BetweenFactor",
					boost::serialization::base_object<BetweenFactor<VALUES, KEY> >(*this));
		}
	}; // \class BetweenConstraint

} /// namespace gtsam
