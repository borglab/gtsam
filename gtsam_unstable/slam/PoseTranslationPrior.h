/**
 * @file PoseTranslationPrior.h
 *
 * @brief Implements a prior on the translation component of a pose
 * 
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/slam/PartialPriorFactor.h>
#include <gtsam/geometry/concepts.h>

namespace gtsam {

template<class POSE>
class PoseTranslationPrior : public PartialPriorFactor<POSE> {
public:

	typedef PoseTranslationPrior<POSE> This;
	typedef PartialPriorFactor<POSE> Base;
	typedef POSE Pose;
	typedef typename POSE::Translation Translation;

	GTSAM_CONCEPT_POSE_TYPE(Pose)
	GTSAM_CONCEPT_GROUP_TYPE(Pose)
	GTSAM_CONCEPT_LIE_TYPE(Translation)

	/** standard constructor */
	PoseTranslationPrior(Key key, const Translation& trans_z, const SharedNoiseModel& model)
	: Base(key, model) {
		initialize(trans_z);
	}

	/** Constructor that pulls the translation from an incoming POSE */
	PoseTranslationPrior(Key key, const POSE& pose_z, const SharedNoiseModel& model)
	: Base(key, model) {
		initialize(pose_z.translation());
	}

	/** get the rotation used to create the measurement */
	Translation priorTranslation() const { return Translation::Expmap(this->prior_); }

protected:
	/** loads the underlying partial prior factor */
	void initialize(const Translation& trans_z) {
		assert(trans_z.dim() == this->noiseModel_->dim());

		// Calculate the prior applied
		this->prior_ = Translation::Logmap(trans_z);

		// Create the mask for partial prior
		size_t pose_dim = Pose::identity().dim();
		size_t rot_dim = trans_z.dim();

		// get the interval of the lie coordinates corresponding to rotation
		std::pair<size_t, size_t> interval = Pose::translationInterval();

		std::vector<size_t> mask;
		for (size_t i=interval.first; i<=interval.second; ++i)
			mask.push_back(i);
		this->mask_ = mask;

		this->H_ = zeros(rot_dim, pose_dim);
		this->fillH();
	}

};

} // \namespace gtsam




