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

#include <gtsam/geometry/StereoCamera.h>

namespace gtsam {

template<class VALUES, class KEY1, class KEY2>
class GenericStereoFactor: public NonlinearFactor2<VALUES, KEY1, KEY2> {
private:

	// Keep a copy of measurement and calibration for I/O
	StereoPoint2 z_;											///< the measurement
	boost::shared_ptr<Cal3_S2Stereo> K_;	///< shared pointer to calibration

public:

	// shorthand for base class type
	typedef NonlinearFactor2<VALUES, KEY1, KEY2> Base;					///< typedef for base class
	typedef boost::shared_ptr<GenericStereoFactor> shared_ptr;  ///< typedef for shared pointer to this object
	typedef typename KEY1::Value CamPose;												///< typedef for Pose Lie Value type

	/**
	 * Default constructor
	 */
	GenericStereoFactor() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)) {}

	/**
	 * Constructor
	 * @param z is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
	 * @param model is the noise model in on the measurement
	 * @param j_pose the pose index
	 * @param j_landmark the landmark index
	 * @param K the constant calibration
	 */
	GenericStereoFactor(const StereoPoint2& z, const SharedNoiseModel& model, KEY1 j_pose,
			KEY2 j_landmark, const shared_ptrKStereo& K) :
		Base(model, j_pose, j_landmark), z_(z), K_(K) {
	}

	~GenericStereoFactor() {}  ///< desctructor

	/**
	 * print
	 * @param s optional string naming the factor
	 */
	void print(const std::string& s) const {
		Base::print(s);
		z_.print(s + ".z");
	}

	/**
	 * equals
	 */
	bool equals(const GenericStereoFactor& f, double tol) const {
		const GenericStereoFactor* p = dynamic_cast<const GenericStereoFactor*> (&f);
		if (p == NULL)
			goto fail;
		//if (cameraFrameNumber_ != p->cameraFrameNumber_ || landmarkNumber_ != p->landmarkNumber_) goto fail;
		if (!z_.equals(p->z_, tol))
			goto fail;
		return true;

		fail: print("actual");
		p->print("expected");
		return false;
	}

	/** h(x)-z */
	Vector evaluateError(const Pose3& pose, const Point3& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
		StereoCamera stereoCam(pose, K_);
		return (stereoCam.project(point, H1, H2) - z_).vector();
	}

	/// get the measurement z
	StereoPoint2 z() {
		return z_;
	}

	/** return the measured */
	inline const StereoPoint2 measured() const {
	  return z_;
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
		ar & BOOST_SERIALIZATION_NVP(z_);
		ar & BOOST_SERIALIZATION_NVP(K_);
	}
};


}
