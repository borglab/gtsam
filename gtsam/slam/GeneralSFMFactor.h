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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

	/**
	 * Non-linear factor for a constraint derived from a 2D measurement.
	 * The calibration is unknown here compared to GenericProjectionFactor
	 * @addtogroup SLAM
	 */
	template <class CAMERA, class LANDMARK>
	class GeneralSFMFactor:	public NoiseModelFactor2<CAMERA, LANDMARK> {
	protected:
		Point2 measured_;			///< the 2D measurement

	public:

		typedef CAMERA Cam;					            	///< typedef for camera type
		typedef GeneralSFMFactor<CAMERA, LANDMARK> This;	///< typedef for this object
		typedef NoiseModelFactor2<CAMERA, LANDMARK> Base;	///< typedef for the base class
		typedef Point2 Measurement;							///< typedef for the measurement

		// shorthand for a smart pointer to a factor
		typedef boost::shared_ptr<This> shared_ptr;

		/**
		 * Constructor
		 * @param measured is the 2 dimensional location of point in image (the measurement)
		 * @param model is the standard deviation of the measurements
		 * @param cameraKey is the index of the camera
		 * @param landmarkKey is the index of the landmark
		 */
		GeneralSFMFactor(const Point2& measured, const SharedNoiseModel& model, Key cameraKey, Key landmarkKey) :
		  Base(model, cameraKey, landmarkKey), measured_(measured) {}

		GeneralSFMFactor():measured_(0.0,0.0) {} 							///< default constructor
		GeneralSFMFactor(const Point2 & p):measured_(p) {}			///< constructor that takes a Point2
		GeneralSFMFactor(double x, double y):measured_(x,y) {} ///< constructor that takes doubles x,y to make a Point2

		virtual ~GeneralSFMFactor() {} ///< destructor

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/**
		 * print
		 * @param s optional string naming the factor
		 * @param keyFormatter optional formatter for printing out Symbols
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
		Vector evaluateError(const Cam& camera,	const Point3& point,
				boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const {

			try {
				Point2 reprojError(camera.project2(point,H1,H2) - measured_);
	      return reprojError.vector();
			}
			catch( CheiralityException& e) {
			  if (H1) *H1 = zeros(2, camera.dim());
			  if (H2) *H2 = zeros(2, point.dim());
//			  cout << e.what() << ": Landmark "<< this->key2_.index()
//			  								 << " behind Camera " << this->key1_.index() << endl;
			}
		  return zero(2);
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
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	};

  /**
   * Non-linear factor for a constraint derived from a 2D measurement.
   * Compared to GeneralSFMFactor, it is a ternary-factor because the calibration is isolated from camera..
   */
  template <class CALIBRATION>
  class GeneralSFMFactor2: public NoiseModelFactor3<Pose3, Point3, CALIBRATION> {
  protected:
    Point2 measured_;     ///< the 2D measurement

  public:

    typedef GeneralSFMFactor2<CALIBRATION> This;
    typedef PinholeCamera<CALIBRATION> Camera;                  ///< typedef for camera type
    typedef NoiseModelFactor3<Pose3, Point3, CALIBRATION> Base; ///< typedef for the base class
    typedef Point2 Measurement;                                 ///< typedef for the measurement

    // shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /**
     * Constructor
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation of the measurements
     * @param poseKey is the index of the camera
     * @param landmarkKey is the index of the landmark
     * @param calibKey is the index of the calibration
     */
    GeneralSFMFactor2(const Point2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, Key calibKey) :
      Base(model, poseKey, landmarkKey, calibKey), measured_(measured) {}
    GeneralSFMFactor2():measured_(0.0,0.0) {}              ///< default constructor

    virtual ~GeneralSFMFactor2() {} ///< destructor

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "SFMFactor2", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      Base::print(s, keyFormatter);
      measured_.print(s + ".z");
    }

    /**
     * equals
     */
    bool equals(const NonlinearFactor &p, double tol = 1e-9) const  {
      const This* e = dynamic_cast<const This*>(&p);
      return e && Base::equals(p, tol) && this->measured_.equals(e->measured_, tol) ;
    }

    /** h(x)-z */
    Vector evaluateError(const Pose3& pose3, const Point3& point, const CALIBRATION &calib,
                         boost::optional<Matrix&> H1=boost::none,
                         boost::optional<Matrix&> H2=boost::none,
                         boost::optional<Matrix&> H3=boost::none) const
    {
      try {
        Camera camera(pose3,calib);
        Point2 reprojError(camera.project(point, H1, H2, H3) - measured_);
        return reprojError.vector();
      }
      catch( CheiralityException& e) {
        if (H1) *H1 = zeros(2, pose3.dim());
        if (H2) *H2 = zeros(2, point.dim());
        if (H3) *H3 = zeros(2, calib.dim());
      }
      return zero(2);
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
      ar & boost::serialization::make_nvp("NoiseModelFactor3",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  };



} //namespace
