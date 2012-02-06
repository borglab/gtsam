/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactor.h
 * @brief Basic bearing factor from 2D measurement
 * @author Chris Beall
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace gtsam {

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
	 * i.e. the main building block for visual SLAM.
	 */
	template<class POSE, class LANDMARK>
	class GenericProjectionFactor: public NonlinearFactor2<POSE, LANDMARK> {
	protected:

		// Keep a copy of measurement and calibration for I/O
		Point2 measured_;					  		///< 2D measurement
		boost::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object

	public:

		/// shorthand for base class type
		typedef NonlinearFactor2<POSE, LANDMARK> Base;

		/// shorthand for this class
		typedef GenericProjectionFactor<POSE, LANDMARK> This;

		/// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<This> shared_ptr;

		/// Default constructor
		GenericProjectionFactor() :
				K_(new Cal3_S2(444, 555, 666, 777, 888)) {
		}

		/**
		 * Constructor
		 * TODO: Mark argument order standard (keys, measurement, parameters)
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation
		 * @param j_pose is basically the frame number
		 * @param j_landmark is the index of the landmark
		 * @param K shared pointer to the constant calibration
		 */
		GenericProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
				const Symbol poseKey, const Symbol& pointKey, const shared_ptrK& K) :
				  Base(model, poseKey, pointKey), measured_(measured), K_(K) {
		}

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "ProjectionFactor") const {
			Base::print(s);
			measured_.print(s + ".z");
		}

		/// equals
		virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*> (&expected);
			return e && Base::equals(p, tol) && this->measured_.equals(e->z_, tol) && this->K_->equals(*e->K_, tol);
		}

		/// Evaluate error h(x)-z and optionally derivatives
		Vector evaluateError(const Pose3& pose, const Point3& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			try {
	      SimpleCamera camera(*K_, pose);
			  Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
	      return reprojectionError.vector();
			} catch( CheiralityException& e) {
			  if (H1) *H1 = zeros(2,6);
			  if (H2) *H2 = zeros(2,3);
			  cout << e.what() << ": Landmark "<< this->key2_.index() <<
			      " moved behind camera " << this->key1_.index() << endl;
			  return ones(2) * 2.0 * K_->fx();
			}
		}

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

	private:

		/// Serialization function
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
			ar & BOOST_SERIALIZATION_NVP(measured_);
			ar & BOOST_SERIALIZATION_NVP(K_);
		}
	};
} // \ namespace gtsam
