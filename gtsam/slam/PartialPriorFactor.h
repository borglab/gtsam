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
	 *   Values where the T's are stored, typically LieValues<Key> or a TupleValues<...>
	 *
	 * For practical use, it would be good to subclass this factor and have the class type
	 * construct the mask.
	 */
	template<class VALUES, class KEY>
	class PartialPriorFactor: public NonlinearFactor1<VALUES, KEY> {

	public:
		typedef typename KEY::Value T;

	protected:

		typedef NonlinearFactor1<VALUES, KEY> Base;
		typedef PartialPriorFactor<VALUES, KEY> This;

		Vector prior_;            /// measurement on logmap parameters, in compressed form
		std::vector<bool> mask_;  /// flags to mask all parameters not measured

		/** default constructor - only use for serialization */
		PartialPriorFactor() {}

		/**
		 * constructor with just minimum requirements for a factor - allows more
		 * computation in the constructor.  This should only be used by subclasses
		 * Sets the size of the mask with all values off
		 */
		PartialPriorFactor(const KEY& key, const SharedNoiseModel& model)
		  : Base(model, key), mask_(T::Dim(), false) {}

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PartialPriorFactor> shared_ptr;

		virtual ~PartialPriorFactor() {}

		/** Full Constructor: requires mask and vector - not for typical use */
		PartialPriorFactor(const KEY& key, const std::vector<bool>& mask,
				const Vector& prior, const SharedNoiseModel& model) :
			Base(model, key), prior_(prior), mask_(mask) {
			assert(mask_.size() == T::Dim()); // NOTE: assumes constant size variable
			assert(nrTrue() == model->dim());
			assert(nrTrue() == prior_.size());
		}

		/** Single Element Constructor: acts on a single parameter specified by idx */
		PartialPriorFactor(const KEY& key, size_t idx, double prior, const SharedNoiseModel& model) :
			Base(model, key), prior_(Vector_(1, prior)), mask_(T::Dim(), false) {
			assert(model->dim() == 1);
			mask_[idx] = true;
			assert(nrTrue() == 1);
		}

		/** Indices Constructor: specify the mask with a set of indices */
		PartialPriorFactor(const KEY& key, const std::vector<size_t>& mask, const Vector& prior,
				const SharedNoiseModel& model) :
			Base(model, key), prior_(prior), mask_(T::Dim(), false) {
			assert((size_t)prior_.size() == mask.size());
			assert(model->dim() == (size_t) prior.size());
			setMask(mask);
			assert(nrTrue() == this->dim());
		}

		/** implement functions needed for Testable */

		/** print */
		virtual void print(const std::string& s) const {
			Base::print(s);
			gtsam::print(prior_, "prior");
		}

		/** equals */
		virtual bool equals(const NonlinearFactor<VALUES>& expected, double tol=1e-9) const {
			const This *e = dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&
					gtsam::equal_with_abs_tol(this->prior_, e->prior_, tol) &&
					this->mask_ == e->mask_;
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = zeros(this->dim(), p.dim());
			// FIXME: this was originally the generic retraction - may not produce same results
			Vector full_logmap = T::Logmap(p);
			Vector masked_logmap = zero(this->dim());
			size_t masked_idx=0;
			for (size_t i=0;i<mask_.size();++i)
				if (mask_[i]) {
					masked_logmap(masked_idx) = full_logmap(i);
					if (H) (*H)(masked_idx, i) = 1.0;
					++masked_idx;
				}
			return masked_logmap - prior_;
		}

		// access
		const Vector& prior() const { return prior_; }
		const std::vector<bool>& mask() const { return  mask_; }

	protected:

		/** counts true elements in the mask */
		size_t nrTrue() const {
			size_t result=0;
			for (size_t i=0; i<mask_.size(); ++i)
				if (mask_[i])	++result;
			return result;
		}

		/** sets the mask using a set of indices */
		void setMask(const std::vector<size_t>& mask) {
			for (size_t i=0; i<mask.size(); ++i) {
				assert(mask[i] < mask_.size());
				mask_[mask[i]] = true;
			}
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
		}
	}; // \class PartialPriorFactor

} /// namespace gtsam
