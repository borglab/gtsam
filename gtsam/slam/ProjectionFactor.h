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
	template<class VALUES, class LMK, class POSK>
	class GenericProjectionFactor: public NonlinearFactor2<VALUES, POSK, LMK> {
	protected:

		// Keep a copy of measurement and calibration for I/O
		Point2 z_;											///< 2D measurement
		boost::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object

	public:

		/// shorthand for base class type
		typedef NonlinearFactor2<VALUES, POSK, LMK> Base;

		/// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<GenericProjectionFactor<VALUES, LMK, POSK> > shared_ptr;

		/// Default constructor
		GenericProjectionFactor() :
				K_(new Cal3_S2(444, 555, 666, 777, 888)) {
		}

		/**
		 * Constructor
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation
		 * @param j_pose is basically the frame number
		 * @param j_landmark is the index of the landmark
		 * @param K shared pointer to the constant calibration
		 */
		GenericProjectionFactor(const Point2& z, const SharedNoiseModel& model,
				POSK j_pose, LMK j_landmark, const shared_ptrK& K) :
				Base(model, j_pose, j_landmark), z_(z), K_(K) {
		}

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "ProjectionFactor") const {
			Base::print(s);
			z_.print(s + ".z");
		}

		/// equals
		bool equals(const GenericProjectionFactor<VALUES, LMK, POSK>& p
				, double tol = 1e-9) const {
			return Base::equals(p, tol) && this->z_.equals(p.z_, tol)
					&& this->K_->equals(*p.K_, tol);
		}

		/// Evaluate error h(x)-z and optionally derivatives
		Vector evaluateError(const Pose3& pose, const Point3& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			try {
	      SimpleCamera camera(*K_, pose);
			  Point2 reprojectionError(camera.project(point, H1, H2) - z_);
	      return reprojectionError.vector();
			}
			catch( CheiralityException& e) {
			  if (H1) *H1 = zeros(2,6);
			  if (H2) *H2 = zeros(2,3);
			  cout << e.what() << ": Landmark "<< this->key2_.index() <<
			      " moved behind camera " << this->key1_.index() << endl;
			  return ones(2) * 2.0 * K_->fx();
			}
		}

    /** return the measurement */
    inline const Point2 measured() const {
      return z_;
    }

	private:

		/// Serialization function
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
			ar & BOOST_SERIALIZATION_NVP(z_);
			ar & BOOST_SERIALIZATION_NVP(K_);
		}
	};
} // \ namespace gtsam
