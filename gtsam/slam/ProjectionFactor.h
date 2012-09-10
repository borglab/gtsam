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
	 * @addtogroup SLAM
	 */
	template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
	class GenericProjectionFactor: public NoiseModelFactor2<POSE, LANDMARK> {
	protected:

		// Keep a copy of measurement and calibration for I/O
		Point2 measured_;					  		    ///< 2D measurement
		boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

	public:

		/// shorthand for base class type
		typedef NoiseModelFactor2<POSE, LANDMARK> Base;

		/// shorthand for this class
		typedef GenericProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

		/// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<This> shared_ptr;

		/// Default constructor
		GenericProjectionFactor() {
		}

		/**
		 * Constructor
		 * TODO: Mark argument order standard (keys, measurement, parameters)
		 * @param measured is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation
		 * @param poseKey is the index of the camera
		 * @param pointKey is the index of the landmark
		 * @param K shared pointer to the constant calibration
		 */
		GenericProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
				Key poseKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K) :
				  Base(model, poseKey, pointKey), measured_(measured), K_(K) {
		}

		/** Virtual destructor */
		virtual ~GenericProjectionFactor() {}

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
      std::cout << s << "GenericProjectionFactor, z = ";
      measured_.print();
			Base::print("", keyFormatter);
		}

		/// equals
		virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
			return e && Base::equals(p, tol) && this->measured_.equals(e->measured_, tol) && this->K_->equals(*e->K_, tol);
		}

		/// Evaluate error h(x)-z and optionally derivatives
		Vector evaluateError(const Pose3& pose, const Point3& point,
				boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			try {
	      PinholeCamera<CALIBRATION> camera(pose, *K_);
			  Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
	      return reprojectionError.vector();
			} catch( CheiralityException& e) {
			  if (H1) *H1 = zeros(2,6);
			  if (H2) *H2 = zeros(2,3);
			  std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
			      " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
			}
		  return ones(2) * 2.0 * K_->fx();
		}

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
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
