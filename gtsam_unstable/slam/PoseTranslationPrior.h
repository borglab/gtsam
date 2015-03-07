/**
 * @file PoseTranslationPrior.h
 *
 * @brief Implements a prior on the translation component of a pose
 * 
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/** simple pose traits for building factors */
namespace pose_traits {

/** checks whether parameterization of rotation is before or after translation in Lie algebra */
template<class POSE>
bool isRotFirst() {
	throw std::invalid_argument("PoseTrait: no implementation for this pose type");
	return false;
}

// Instantiate for common poses
template<> inline bool isRotFirst<Pose3>() { return true; }
template<> inline bool isRotFirst<Pose2>() { return false; }

} // \namespace pose_traits

/**
 * A prior on the translation part of a pose
 */
template<class POSE>
class PoseTranslationPrior : public NoiseModelFactor1<POSE> {
public:
	typedef PoseTranslationPrior<POSE> This;
	typedef NoiseModelFactor1<POSE> Base;
	typedef POSE Pose;
	typedef typename POSE::Translation Translation;
	typedef typename POSE::Rotation Rotation;

	GTSAM_CONCEPT_POSE_TYPE(Pose)
	GTSAM_CONCEPT_GROUP_TYPE(Pose)
	GTSAM_CONCEPT_LIE_TYPE(Translation)

protected:

	Translation measured_;

public:

	/** standard constructor */
	PoseTranslationPrior(Key key, const Translation& measured, const noiseModel::Base::shared_ptr& model)
	: Base(model, key), measured_(measured) {
	}

	/** Constructor that pulls the translation from an incoming POSE */
	PoseTranslationPrior(Key key, const POSE& pose_z, const noiseModel::Base::shared_ptr& model)
	: Base(model, key), measured_(pose_z.translation()) {
	}

	virtual ~PoseTranslationPrior() {}

	const Translation& measured() const { return measured_; }

	/// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
	  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
	      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

	/** h(x)-z */
	Vector evaluateError(const Pose& pose, boost::optional<Matrix&> H = boost::none) const {
		const Translation& newTrans = pose.translation();
		const Rotation& R = pose.rotation();
		const size_t tDim = newTrans.dim(), xDim = pose.dim();
		if (H) {
			*H = gtsam::zeros(tDim, xDim);
			if (pose_traits::isRotFirst<Pose>())
				(*H).rightCols(tDim) = R.matrix();
			else
				(*H).leftCols(tDim) = R.matrix();
		}

		return newTrans.vector() - measured_.vector();
	}

	/** equals specialized to this factor */
	virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
		const This *e = dynamic_cast<const This*> (&expected);
		return e != NULL && Base::equals(*e, tol) && measured_.equals(e->measured_, tol);
	}

	/** print contents */
	void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
		Base::print(s + "PoseTranslationPrior", keyFormatter);
		measured_.print("Measured Translation");
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




