/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericStereoFactor.h
 * @brief   A non-linear factor for stereo measurements
 * @author  Alireza Fathi
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/StereoCamera.h>

namespace gtsam {

/**
 * A Generic Stereo Factor
 * @addtogroup SLAM
 */
template<class POSE, class LANDMARK>
class GenericStereoFactor: public NoiseModelFactor2<POSE, LANDMARK> {
private:

	// Keep a copy of measurement and calibration for I/O
	StereoPoint2 measured_;											///< the measurement
	boost::shared_ptr<Cal3_S2Stereo> K_;	///< shared pointer to calibration

public:

	// shorthand for base class type
	typedef NoiseModelFactor2<POSE, LANDMARK> Base;		     			///< typedef for base class
	typedef GenericStereoFactor<POSE, LANDMARK> This;           ///< typedef for this class (with templates)
	typedef boost::shared_ptr<GenericStereoFactor> shared_ptr;  ///< typedef for shared pointer to this object
	typedef POSE CamPose;												///< typedef for Pose Lie Value type

	/**
	 * Default constructor
	 */
	GenericStereoFactor() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)) {}

	/**
	 * Constructor
	 * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
	 * @param model is the noise model in on the measurement
	 * @param poseKey the pose variable key
	 * @param landmarkKey the landmark variable key
	 * @param K the constant calibration
	 */
	GenericStereoFactor(const StereoPoint2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, const shared_ptrKStereo& K) :
		Base(model, poseKey, landmarkKey), measured_(measured), K_(K) {
	}

	virtual ~GenericStereoFactor() {}  ///< Virtual destructor

	/// @return a deep copy of this factor
	virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

	/**
	 * print
	 * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
	 */
	void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
		Base::print(s, keyFormatter);
		measured_.print(s + ".z");
	}

	/**
	 * equals
	 */
	virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
	  const GenericStereoFactor* p = dynamic_cast<const GenericStereoFactor*> (&f);
		return p && Base::equals(f) && measured_.equals(p->measured_, tol);
	}

	/** h(x)-z */
	Vector evaluateError(const Pose3& pose, const Point3& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
		StereoCamera stereoCam(pose, K_);
		return (stereoCam.project(point, H1, H2) - measured_).vector();
	}

	/** return the measured */
	const StereoPoint2& measured() const {
	  return measured_;
	}

  /** return the calibration object */
  inline const Cal3_S2Stereo::shared_ptr calibration() const {
    return K_;
  }


private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(measured_);
		ar & BOOST_SERIALIZATION_NVP(K_);
	}
};


}
