/**
 * @file PoseRotationPrior.h
 *
 * @brief Implements a prior on the rotation component of a pose
 * 
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/slam/PartialPriorFactor.h>
#include <gtsam/geometry/concepts.h>

namespace gtsam {

template<class POSE>
class PoseRotationPrior : public PartialPriorFactor<POSE> {
public:

	typedef PoseRotationPrior<POSE> This;
	typedef PartialPriorFactor<POSE> Base;
	typedef POSE Pose;
	typedef typename POSE::Rotation Rotation;

	GTSAM_CONCEPT_POSE_TYPE(Pose)
	GTSAM_CONCEPT_GROUP_TYPE(Pose)
	GTSAM_CONCEPT_LIE_TYPE(Rotation)

	/** standard constructor */
	PoseRotationPrior(Key key, const Rotation& rot_z, const SharedNoiseModel& model)
	: Base(key, model) {
		initialize(rot_z);
	}

	/** Constructor that pulls the translation from an incoming POSE */
	PoseRotationPrior(Key key, const POSE& pose_z, const SharedNoiseModel& model)
	: Base(key, model) {
		initialize(pose_z.rotation());
	}

	/** get the rotation used to create the measurement */
	Rotation priorRotation() const { return Rotation::Expmap(this->prior_); }

protected:
	/** loads the underlying partial prior factor */
	void initialize(const Rotation& rot_z) {
		assert(rot_z.dim() == this->noiseModel_->dim());

		// Calculate the prior applied
		this->prior_ = Rotation::Logmap(rot_z);

		// Create the mask for partial prior
		size_t pose_dim = Pose::identity().dim();
		size_t rot_dim = rot_z.dim();

		// get the interval of the lie coordinates corresponding to rotation
		std::pair<size_t, size_t> interval = Pose::rotationInterval();

		std::vector<size_t> mask;
		for (size_t i=interval.first; i<=interval.second; ++i)
			mask.push_back(i);
		this->mask_ = mask;

		this->H_ = zeros(rot_dim, pose_dim);
		this->fillH();
	}

};

} // \namespace gtsam




