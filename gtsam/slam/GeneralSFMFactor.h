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
	template <class VALUES, class CamK, class LmK>
	class GeneralSFMFactor:
	public NonlinearFactor2<VALUES, CamK, LmK> {
	protected:
		Point2 z_;			///< the 2D measurement

	public:

		typedef typename CamK::Value Cam;										///< typedef for camera type
		typedef GeneralSFMFactor<VALUES, CamK, LmK> Self ;	///< typedef for this object
		typedef NonlinearFactor2<VALUES, CamK, LmK> Base;		///< typedef for the base class
		typedef Point2 Measurement;													///< typedef for the measurement

		// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<GeneralSFMFactor<VALUES, LmK, CamK> > shared_ptr;

		/**
		 * Constructor
		 * @param z is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation of the measurements
		 * @param i is basically the frame number
		 * @param j is the index of the landmark
		 */
		GeneralSFMFactor(const Point2& z, const SharedNoiseModel& model, const CamK& i, const LmK& j) : Base(model, i, j), z_(z) {}

		GeneralSFMFactor():z_(0.0,0.0) {} 							///< default constructor
		GeneralSFMFactor(const Point2 & p):z_(p) {}			///< constructor that takes a Point2
		GeneralSFMFactor(double x, double y):z_(x,y) {} ///< constructor that takes doubles x,y to make a Point2

		virtual ~GeneralSFMFactor() {} ///< destructor

		/**
		 * print
		 * @param s optional string naming the factor
		 */
		void print(const std::string& s = "SFMFactor") const {
			Base::print(s);
			z_.print(s + ".z");
		}

		/**
		 * equals
		 */
		bool equals(const GeneralSFMFactor<VALUES, CamK, LmK> &p, double tol = 1e-9) const	{
			return Base::equals(p, tol) && this->z_.equals(p.z_, tol) ;
		}

		/** h(x)-z */
		Vector evaluateError(const Cam& camera,	const Point3& point,
				boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const {

			try {
				Point2 reprojError(camera.project2(point,H1,H2) - z_);
	      return reprojError.vector();
			}
			catch( CheiralityException& e) {
			  if (H1) *H1 = zeros(2, camera.dim());
			  if (H2) *H2 = zeros(2, point.dim());
//			  cout << e.what() << ": Landmark "<< this->key2_.index()
//			  								 << " behind Camera " << this->key1_.index() << endl;
			  return zero(2);
			}
		}

		/** return the measured */
		inline const Point2 measured() const {
			return z_;
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(z_);
		}
	};

} //namespace
