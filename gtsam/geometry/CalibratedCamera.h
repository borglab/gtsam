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
	 * @addtogroup geometry
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
		explicit CalibratedCamera(const Pose3& pose);

    /// @}
    /// @name Advanced Constructors
    /// @{

		/// construct from vector
		explicit CalibratedCamera(const Vector &v);

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
		 * @param height specifies the height of the camera (along the positive Z-axis)
		 * (theta 0 = looking in direction of positive X axis)
		 */
		static CalibratedCamera Level(const Pose2& pose2, double height);

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
    /// @name Transformations and mesaurement functions
    /// @{

		/**
		 * This function receives the camera pose and the landmark location and
		 * returns the location the point is supposed to appear in the image
		 * @param point a 3D point to be projected
		 * @param D_intrinsic_pose the optionally computed Jacobian with respect to pose
		 * @param D_intrinsic_point the optionally computed Jacobian with respect to the 3D point
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

    /**
     * Calculate range to a landmark
     * @param point 3D location of landmark
     * @param H1 optionally computed Jacobian with respect to pose
     * @param H2 optionally computed Jacobian with respect to the 3D point
     * @return range (double)
     */
    double range(const Point3& point,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      return pose_.range(point, H1, H2); }

    /**
     * Calculate range to another pose
     * @param pose Other SO(3) pose
     * @param H1 optionally computed Jacobian with respect to pose
     * @param H2 optionally computed Jacobian with respect to the 3D point
     * @return range (double)
     */
    double range(const Pose3& pose,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      return pose_.range(pose, H1, H2); }

    /**
     * Calculate range to another camera
     * @param camera Other camera
     * @param H1 optionally computed Jacobian with respect to pose
     * @param H2 optionally computed Jacobian with respect to the 3D point
     * @return range (double)
     */
    double range(const CalibratedCamera& camera,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const {
      return pose_.range(camera.pose_, H1, H2); }

private:

    /// @}
    /// @name Advanced Interface
    /// @{

	    /** Serialization function */
	    friend class boost::serialization::access;
	    template<class Archive>
	    void serialize(Archive & ar, const unsigned int version) {
	  		ar & boost::serialization::make_nvp("CalibratedCamera",
	  				boost::serialization::base_object<Value>(*this));
	      ar & BOOST_SERIALIZATION_NVP(pose_);
	    }

	    /// @}
	};
}

