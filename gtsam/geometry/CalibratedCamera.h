/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CalibratedCamera.h
 * @brief Calibrated camera for which only pose is unknown
 * @date Aug 17, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  class CheiralityException: public std::runtime_error {
  public:
    CheiralityException() : std::runtime_error("Cheirality Exception") {}
  };

	/**
	 * A Calibrated camera class [R|-R't], calibration K=I.
	 * If calibration is known, it is more computationally efficient
	 * to calibrate the measurements rather than try to predict in pixels.
	 * @ingroup geometry
	 * \nosubgrouping
	 */
	class CalibratedCamera : public DerivedValue<CalibratedCamera> {
	private:
		Pose3 pose_; // 6DOF pose

	public:

    /// @name Standard Constructors
    /// @{

		/// default constructor
		CalibratedCamera() {}

		/// construct with pose
		CalibratedCamera(const Pose3& pose);

    /// @}
    /// @name Advanced Constructors
    /// @{

		/// construct from vector
		CalibratedCamera(const Vector &v);

		/// @}
		/// @name Testable
		/// @{

		virtual void print(const std::string& s = "") const {
			pose_.print(s);
		}

		/// check equality to another camera
		bool equals (const CalibratedCamera &camera, double tol = 1e-9) const {
			return pose_.equals(camera.pose(), tol) ;
		}

    /// @}
    /// @name Standard Interface
    /// @{

		/// destructor
		virtual ~CalibratedCamera() {}

		/// return pose
		inline const Pose3& pose() const {	return pose_; }

		/// compose the poses
		inline const CalibratedCamera compose(const CalibratedCamera &c) const {
			return CalibratedCamera( pose_ * c.pose() ) ;
		}

		/// invert the camera's pose
		inline const CalibratedCamera inverse() const {
			return CalibratedCamera( pose_.inverse() ) ;
		}

		/**
		 * Create a level camera at the given 2D pose and height
		 * @param pose2 specifies the location and viewing direction
		 * (theta 0 = looking in direction of positive X axis)
		 */
		static CalibratedCamera level(const Pose2& pose2, double height);

		/// @}
		/// @name Manifold
		/// @{

		/// move a cameras pose according to d
		CalibratedCamera retract(const Vector& d) const;

		/// Return canonical coordinate
	  Vector localCoordinates(const CalibratedCamera& T2) const;

	  /// Lie group dimensionality
		inline size_t dim() const { return 6 ; }

	  /// Lie group dimensionality
		inline static size_t Dim() { return 6 ; }


		/* ************************************************************************* */
		// measurement functions and derivatives
		/* ************************************************************************* */

    /// @}
    /// @name Transformations
    /// @{

		/**
		 * This function receives the camera pose and the landmark location and
		 * returns the location the point is supposed to appear in the image
		 * @param camera the CalibratedCamera
		 * @param point a 3D point to be projected
		 * @return the intrinsic coordinates of the projected point
		 */
		Point2 project(const Point3& point,
			    boost::optional<Matrix&> D_intrinsic_pose = boost::none,
			    boost::optional<Matrix&> D_intrinsic_point = boost::none) const;

		/**
		 * projects a 3-dimensional point in camera coordinates into the
		 * camera and returns a 2-dimensional point, no calibration applied
		 * With optional 2by3 derivative
		 */
		static Point2 project_to_camera(const Point3& cameraPoint,
				boost::optional<Matrix&> H1 = boost::none);

		/**
		 * backproject a 2-dimensional point to a 3-dimension point
		 */
		static Point3 backproject_from_camera(const Point2& p, const double scale);

private:

    /// @}
    /// @name Advanced Interface
    /// @{

	    /** Serialization function */
	    friend class boost::serialization::access;
	    template<class Archive>
	    void serialize(Archive & ar, const unsigned int version) {
	      ar & BOOST_SERIALIZATION_NVP(pose_);
	    }

	    /// @}
	};
}

