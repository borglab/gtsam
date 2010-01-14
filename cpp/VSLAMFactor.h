/**
 * @file    VSLAMFactor.h
 * @brief   A Nonlinear Factor, specialized for visual SLAM
 * @author  Alireza Fathi
 */

#pragma once

#include <boost/optional.hpp>

#include "NonlinearFactor.h"
#include "SimpleCamera.h"
#include "VSLAMConfig.h"
#include "Cal3_S2.h"

namespace gtsam {

	typedef NonlinearFactor2<VSLAMConfig,
	VSLAMPoseKey,	Pose3, VSLAMPointKey, Point3> VSLAMFactorBase;

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement,
	 * i.e. the main building block for visual SLAM.
	 */
	class VSLAMFactor: public VSLAMFactorBase , Testable<VSLAMFactor> {
	private:

		// Keep a copy of measurement and calibration for I/O
		Point2 z_;
		boost::shared_ptr<Cal3_S2> K_;

	public:

		 // shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<VSLAMFactor> shared_ptr;

		/**
		 * Default constructor
		 */
		VSLAMFactor();

		/**
		 * Constructor
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param sigma is the standard deviation
		 * @param cameraFrameNumber is basically the frame number
		 * @param landmarkNumber is the index of the landmark
		 * @param K the constant calibration
		 */
		VSLAMFactor(const Point2& z, double sigma, int cameraFrameNumber,
				int landmarkNumber, const shared_ptrK & K);

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "VSLAMFactor") const;

		/**
		 * equals
		 */
		bool equals(const VSLAMFactor&, double tol = 1e-9) const;

		/** h(x) */
		Point2 predict(const Pose3& pose, const Point3& point) const {
			return SimpleCamera(*K_, pose).project(point);
		}

		/** h(x)-z */
		Vector evaluateError(const Pose3& pose, const Point3& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			SimpleCamera camera(*K_, pose);
			if (H1) *H1=Dproject_pose(camera,point);
			if (H2) *H2=Dproject_point(camera,point);
			Point2 reprojectionError(project(camera, point) - z_);
			return reprojectionError.vector();
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			//ar & BOOST_SERIALIZATION_NVP(key1_);
			//ar & BOOST_SERIALIZATION_NVP(key2_);
			ar & BOOST_SERIALIZATION_NVP(z_);
			ar & BOOST_SERIALIZATION_NVP(K_);
		}
	};

}
