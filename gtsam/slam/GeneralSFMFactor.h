/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GeneralSFMFactor.h
 *
 * @brief a general SFM factor with an unknown calibration
 *
 * @date Dec 15, 2010
 * @author Kai Ni
 */

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


namespace gtsam {

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement. The calibration is unknown here compared to GenericProjectionFactor
	 */
	template <class CAMERA, class LANDMARK>
	class GeneralSFMFactor:
	public NoiseModelFactor2<CAMERA, LANDMARK> {
	protected:
		Point2 measured_;			///< the 2D measurement

	public:

		typedef CAMERA Cam;					            					///< typedef for camera type
		typedef GeneralSFMFactor<CAMERA, LANDMARK> This;	///< typedef for this object
		typedef NoiseModelFactor2<CAMERA, LANDMARK> Base;	///< typedef for the base class
		typedef Point2 Measurement;												///< typedef for the measurement

		// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<This> shared_ptr;

		/**
		 * Constructor
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation of the measurements
		 * @param i is basically the frame number
		 * @param j is the index of the landmark
		 */
		GeneralSFMFactor(const Point2& measured, const SharedNoiseModel& model, Key cameraKey, Key landmarkKey) :
		  Base(model, cameraKey, landmarkKey), measured_(measured) {}

		GeneralSFMFactor():measured_(0.0,0.0) {} 							///< default constructor
		GeneralSFMFactor(const Point2 & p):measured_(p) {}			///< constructor that takes a Point2
		GeneralSFMFactor(double x, double y):measured_(x,y) {} ///< constructor that takes doubles x,y to make a Point2

		virtual ~GeneralSFMFactor() {} ///< destructor

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "SFMFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			Base::print(s, keyFormatter);
			measured_.print(s + ".z");
		}

		/**
		 * equals
		 */
		bool equals(const NonlinearFactor &p, double tol = 1e-9) const	{
		  const This* e = dynamic_cast<const This*>(&p);
			return e && Base::equals(p, tol) && this->measured_.equals(e->measured_, tol) ;
		}

		/** h(x)-z */
		Vector evaluateError(
				const Cam& camera,
				const Point3& point,
				boost::optional<Matrix&> H1=boost::none,
				boost::optional<Matrix&> H2=boost::none) const {

			Vector error = measured_.localCoordinates(camera.project2(point,H1,H2));
//			gtsam::print(error, "error");
			return error;
		}

		/** return the measured */
		inline const Point2 measured() const {
			return measured_;
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	};

} //namespace
