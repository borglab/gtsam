/**
 * @file PoseRotationPrior.h
 *
 * @brief Implements a prior on the rotation component of a pose
 * 
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/slam/PoseTranslationPrior.h>

namespace gtsam {

template<class POSE>
class PoseRotationPrior : public NoiseModelFactor1<POSE> {
public:

	typedef PoseRotationPrior<POSE> This;
	typedef NoiseModelFactor1<POSE> Base;
	typedef POSE Pose;
	typedef typename POSE::Translation Translation;
	typedef typename POSE::Rotation Rotation;

	GTSAM_CONCEPT_POSE_TYPE(Pose)
	GTSAM_CONCEPT_GROUP_TYPE(Pose)
	GTSAM_CONCEPT_LIE_TYPE(Rotation)

protected:

	Rotation measured_;

public:

	/** standard constructor */
	PoseRotationPrior(Key key, const Rotation& rot_z, const SharedNoiseModel& model)
	: Base(model, key), measured_(rot_z) {}

	/** Constructor that pulls the translation from an incoming POSE */
	PoseRotationPrior(Key key, const POSE& pose_z, const SharedNoiseModel& model)
	: Base(model, key), measured_(pose_z.rotation()) {}

	virtual ~PoseRotationPrior() {}

	// access
	const Rotation& measured() const { return measured_; }

	// testable

	/** equals specialized to this factor */
	virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
		const This *e = dynamic_cast<const This*> (&expected);
		return e != NULL && Base::equals(*e, tol) && measured_.equals(e->measured_, tol);
	}

	/** print contents */
	void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
		Base::print(s + "PoseRotationPrior", keyFormatter);
		measured_.print("Measured Rotation");
	}

	/** h(x)-z */
	Vector evaluateError(const Pose& pose, boost::optional<Matrix&> H = boost::none) const {
		const Rotation& newR = pose.rotation();
		const size_t rDim = newR.dim(), xDim = pose.dim();
		if (H) {
			*H = gtsam::zeros(rDim, xDim);
			if (pose_traits::isRotFirst<Pose>())
				(*H).leftCols(rDim).setIdentity(rDim, rDim);
			else
				(*H).rightCols(rDim).setIdentity(rDim, rDim);
		}

		return Rotation::Logmap(newR) - Rotation::Logmap(measured_);
	}

private:

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NoiseModelFactor1",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(measured_);
	}
};

} // \namespace gtsam




