/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PinholeCamera.h
 * @brief Base class for all pinhole cameras
 * @author Yong-Dian Jian
 * @date Jan 27, 2012
 */

#pragma once

#include <cmath>
#include <boost/optional.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

  /**
   * A pinhole camera class that has a Pose3 and a Calibration.
   * @ingroup geometry
   * \nosubgrouping
   */
  template <typename Calibration>
  class PinholeCamera : public DerivedValue<PinholeCamera<Calibration> > {
  private:
    Pose3 pose_;
    Calibration k_;

  public:

	/// @name Standard Constructors
	/// @{

    /** default constructor */
    PinholeCamera() {}

    /** constructor with pose */
    explicit PinholeCamera(const Pose3& pose):pose_(pose){}

    /** constructor with pose and calibration */
    PinholeCamera(const Pose3& pose, const Calibration& k):pose_(pose),k_(k) {}

    /** alternative constructor with pose and calibration */
    PinholeCamera(const Calibration& k, const Pose3& pose):pose_(pose),k_(k) {}

    /// @}
    /// @name Advanced Constructors
    /// @{

    explicit PinholeCamera(const Vector &v){
      pose_ = Pose3::Expmap(v.head(Pose3::Dim()));
      if ( v.size() > Pose3::Dim()) {
        k_ = Calibration(v.tail(Calibration::Dim()));
      }
    }

    PinholeCamera(const Vector &v, const Vector &k) : pose_(Pose3::Expmap(v)),k_(k){}

    /// @}
    /// @name Standard Interface
    /// @{

    virtual ~PinholeCamera() {}

    /// return pose
    inline Pose3& pose() {  return pose_; }

    /// return pose
    inline const Pose3& pose() const {  return pose_; }

    /// return calibration
    inline Calibration& calibration() {  return k_; }

    /// return calibration
    inline const Calibration& calibration() const {  return k_; }

    /// compose two cameras
    inline const PinholeCamera compose(const Pose3 &c) const {
    	return PinholeCamera( pose_ * c, k_ ) ;
    }

    /// inverse
    inline const PinholeCamera inverse() const {
    	return PinholeCamera( pose_.inverse(), k_ ) ;
    }

  	/// @}
  	/// @name Testable
  	/// @{

    /// assert equality up to a tolerance
    bool equals (const PinholeCamera &camera, double tol = 1e-9) const {
    	return pose_.equals(camera.pose(), tol) &&
    	       k_.equals(camera.calibration(), tol) ;
    }

    /// print
    void print(const std::string& s = "") const {
      pose_.print("pose3");
      k_.print("calibration");
    }

  	/// @}
  	/// @name Manifold
  	/// @{

    /// move a cameras according to d
    PinholeCamera retract(const Vector& d) const {
      if ((size_t) d.size() == pose_.dim() )
        return PinholeCamera(pose().retract(d), calibration()) ;
      else
        return PinholeCamera(pose().retract(d.head(pose().dim())),
                             calibration().retract(d.tail(calibration().dim()))) ;
    }

    /// return canonical coordinate
    Vector localCoordinates(const PinholeCamera& T2) const {
      Vector d(dim());
      d.head(pose().dim()) = pose().localCoordinates(T2.pose());
      d.tail(calibration().dim()) = calibration().localCoordinates(T2.calibration());
      return d;
    }

    /// Lie group dimensionality
    inline size_t dim() const { return pose_.dim() + k_.dim(); }

    /// Lie group dimensionality
    inline static size_t Dim() { return Pose3::Dim() + Calibration::Dim(); }

    /**
     * Create a level camera at the given 2D pose and height
     * @param pose2 specifies the location and viewing direction
     * (theta 0 = looking in direction of positive X axis)
     */
    static PinholeCamera level(const Pose2& pose2, double height) {
        return PinholeCamera::level(Calibration(), pose2, height);
    }

    static PinholeCamera level(const Calibration &K, const Pose2& pose2, double height) {
        const double st = sin(pose2.theta()), ct = cos(pose2.theta());
        const Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
        const Rot3 wRc(x, y, z);
        const Point3 t(pose2.x(), pose2.y(), height);
        const Pose3 pose3(wRc, t);
        return PinholeCamera(pose3, K);
    }

    /* ************************************************************************* */
    // measurement functions and derivatives
    /* ************************************************************************* */

    /**
    * projects a 3-dimensional point in camera coordinates into the
    * camera and returns a 2-dimensional point, no calibration applied
    * With optional 2by3 derivative
    */
    inline static Point2 project_to_camera(const Point3& P,
                             boost::optional<Matrix&> H1 = boost::none){
      if (H1) {
        double d = 1.0 / P.z(), d2 = d * d;
        *H1 = Matrix_(2, 3, d, 0.0, -P.x() * d2, 0.0, d, -P.y() * d2);
      }
      return Point2(P.x() / P.z(), P.y() / P.z());
    }

    /// Project a point into the image and check depth
    inline std::pair<Point2,bool> projectSafe(const Point3& pw) const {
      const Point3 pc = pose_.transform_to(pw) ;
      const Point2 pn = project_to_camera(pc) ;
      return std::make_pair(k_.uncalibrate(pn), pc.z()>0);
    }

    /// @}
    /// @name Transformations
    /// @{

    /** project a point from world coordinate to the image
     *  @param pw is a point in the world coordinate
     *  @param H1 is the jacobian w.r.t. pose3
     *  @param H2 is the jacobian w.r.t. point3
     *  @param H3 is the jacobian w.r.t. calibration
     */
    inline Point2 project(const Point3& pw,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none) const {

      if (!H1 && !H2 && !H3) {
        const Point3 pc = pose_.transform_to(pw) ;
        if ( pc.z() <= 0 ) throw CheiralityException();
        const Point2 pn = project_to_camera(pc) ;
        return k_.uncalibrate(pn);
      }

      // world to camera coordinate
      Matrix Hc1 /* 3*6 */, Hc2 /* 3*3 */ ;
      const Point3 pc = pose_.transform_to(pw, Hc1, Hc2) ;
      if( pc.z() <= 0 ) throw CheiralityException();

      // camera to normalized image coordinate
      Matrix Hn; // 2*3
      const Point2 pn = project_to_camera(pc, Hn) ;

      // uncalibration
      Matrix Hk, Hi; // 2*2
      const Point2 pi = k_.uncalibrate(pn, Hk, Hi);

      // chain the jacobian matrices
      const Matrix tmp = Hi*Hn ;
      if (H1) *H1 = tmp * Hc1 ;
      if (H2) *H2 = tmp * Hc2 ;
      if (H3) *H3 = Hk;
      return pi;
    }

    /** project a point from world coordinate to the image
     *  @param pw is a point in the world coordinate
     *  @param H1 is the jacobian w.r.t. [pose3 calibration]
     *  @param H2 is the jacobian w.r.t. point3
     */
    inline Point2 project2(const Point3& pw,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const {

      if (!H1 && !H2) {
        const Point3 pc = pose_.transform_to(pw) ;
        if ( pc.z() <= 0 ) throw CheiralityException();
        const Point2 pn = project_to_camera(pc) ;
        return k_.uncalibrate(pn);
      }

      Matrix Htmp1, Htmp2, Htmp3;
      const Point2 pi = this->project(pw, Htmp1, Htmp2, Htmp3);
      if (H1) {
        *H1 = Matrix(2, this->dim());
        H1->leftCols(pose().dim()) = Htmp1 ;         // jacobian wrt pose3
        H1->rightCols(calibration().dim()) = Htmp3 ; // jacobian wrt calib
      }
      if (H2) *H2 = Htmp2;
      return pi;
    }

    /**
     * backproject a 2-dimensional point to a 3-dimension point
     */

    inline Point3 backproject(const Point2& pi, const double scale) const {
      const Point2 pn = k_.calibrate(pi);
      const Point3 pc(pn.x()*scale, pn.y()*scale, scale);
      return pose_.transform_from(pc);
    }

    inline Point3 backproject_from_camera(const Point2& pi, const double scale) const {
      return backproject(pi, scale);
    }

private:

    /// @}
    /// @name Advanced Interface
    /// @{

      /** Serialization function */
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version) {
      	ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
        ar & BOOST_SERIALIZATION_NVP(pose_);
        ar & BOOST_SERIALIZATION_NVP(k_);
      }
      /// @}
  };
}
