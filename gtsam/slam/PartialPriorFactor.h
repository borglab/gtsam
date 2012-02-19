/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PartialPriorFactor.h
 * @brief A simple prior factor that allows for setting a prior only on a part of linear parameters
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

	/**
	 * A class for a soft partial prior on any Lie type, with a mask over Expmap
	 * parameters. Note that this will use Logmap() to find a tangent space parameterization
	 * for the variable attached, so this may fail for highly nonlinear manifolds.
	 *
	 * The prior vector used in this factor is stored in compressed form, such that
	 * it only contains values for measurements that are to be compared, and they are in
	 * the same order as T::Logmap().  The mask will determine which components to extract
	 * in the error function.
	 *
	 * It takes two template parameters:
	 *   Key (typically TypedSymbol) is used to look up T's in a Values
	 *   Values where the T's are stored, typically Values<Key> or a TupleValues<...>
	 *
	 * For practical use, it would be good to subclass this factor and have the class type
	 * construct the mask.
	 */
	template<class VALUE>
	class PartialPriorFactor: public NonlinearFactor1<VALUE> {

	public:
		typedef VALUE T;

	protected:

		typedef NonlinearFactor1<VALUE> Base;
		typedef PartialPriorFactor<VALUE> This;

		Vector prior_;             ///< measurement on logmap parameters, in compressed form
		std::vector<size_t> mask_; ///< indices of values to constrain in compressed prior vector
		Matrix H_; 								 ///< Constant jacobian - computed at creation

		/** default constructor - only use for serialization */
		PartialPriorFactor() {}

		/**
		 * constructor with just minimum requirements for a factor - allows more
		 * computation in the constructor.  This should only be used by subclasses
		 */
		PartialPriorFactor(Key key, const SharedNoiseModel& model)
		  : Base(model, key) {}

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PartialPriorFactor> shared_ptr;

		virtual ~PartialPriorFactor() {}

		/** Single Element Constructor: acts on a single parameter specified by idx */
		PartialPriorFactor(Key key, size_t idx, double prior, const SharedNoiseModel& model) :
			Base(model, key), prior_(Vector_(1, prior)), mask_(1, idx), H_(zeros(1, T::Dim())) {
			assert(model->dim() == 1);
			this->fillH();
		}

		/** Indices Constructor: specify the mask with a set of indices */
		PartialPriorFactor(Key key, const std::vector<size_t>& mask, const Vector& prior,
				const SharedNoiseModel& model) :
			Base(model, key), prior_(prior), mask_(mask), H_(zeros(mask.size(), T::Dim())) {
			assert((size_t)prior_.size() == mask.size());
			assert(model->dim() == (size_t) prior.size());
			this->fillH();
		}

		/** implement functions needed for Testable */

		/** print */
		virtual void print(const std::string& s) const {
			Base::print(s);
			gtsam::print(prior_, "prior");
		}

		/** equals */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e = dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&
					gtsam::equal_with_abs_tol(this->prior_, e->prior_, tol) &&
					this->mask_ == e->mask_;
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = H_;
			// FIXME: this was originally the generic retraction - may not produce same results
			Vector full_logmap = T::Logmap(p);
			Vector masked_logmap = zero(this->dim());
			for (size_t i=0; i<mask_.size(); ++i)
				masked_logmap(i) = full_logmap(mask_[i]);
			return masked_logmap - prior_;
		}

		// access
		const Vector& prior() const { return prior_; }
		const std::vector<bool>& mask() const { return  mask_; }
		const Matrix& H() const { return H_; }

	protected:

		/** Constructs the jacobian matrix in place */
		void fillH() {
			for (size_t i=0; i<mask_.size(); ++i)
				H_(i, mask_[i]) = 1.0;
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NonlinearFactor1",
					boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(prior_);
			ar & BOOST_SERIALIZATION_NVP(mask_);
			ar & BOOST_SERIALIZATION_NVP(H_);
		}
	}; // \class PartialPriorFactor

} /// namespace gtsam
