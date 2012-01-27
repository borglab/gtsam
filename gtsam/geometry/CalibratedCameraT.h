/**
 * @file CalibratedCameraT.h
 * @date Mar 5, 2011
 * @author Yong-Dian Jian
 * @brief calibrated camera template
 */

#pragma once

#include <boost/optional.hpp>

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  /**
   * A Calibrated camera class [R|-R't], calibration K.
   * If calibration is known, it is more computationally efficient
   * to calibrate the measurements rather than try to predict in pixels.
   * AGC: Is this used or tested anywhere?
   * AGC: If this is a "CalibratedCamera," why is there a calibration stored internally?
   * @ingroup geometry
   * \nosubgrouping
   */
  template <typename Calibration>
  class CalibratedCameraT : public DerivedValue<CalibratedCameraT<Calibration> > {
  private:
    Pose3 pose_; // 6DOF pose
    Calibration k_;

  public:

    /// @name Standard Constructors
    /// @{

    ///TODO: comment
    CalibratedCameraT() {}

    ///TODO: comment
    CalibratedCameraT(const Pose3& pose):pose_(pose){}

    ///TODO: comment
    CalibratedCameraT(const Pose3& pose, const Calibration& k):pose_(pose),k_(k) {}

    /// @}
    /// @name Advanced Constructors
    /// @{

    ///TODO: comment
    CalibratedCameraT(const Vector &v): pose_(Pose3::Expmap(v)) {}

    ///TODO: comment
    CalibratedCameraT(const Vector &v, const Vector &k):pose_(Pose3::Expmap(v)),k_(k){}

    /// @}
    /// @name Standard Interface
    /// @{

    virtual ~CalibratedCameraT() {}

    ///TODO: comment
    inline Pose3& pose() {  return pose_; }

    ///TODO: comment
    inline const Pose3& pose() const {  return pose_; }

    ///TODO: comment
    inline Calibration& calibration() {  return k_; }

    ///TODO: comment
    inline const Calibration& calibration() const {  return k_; }

    ///TODO: comment
    inline const CalibratedCameraT compose(const CalibratedCameraT &c) const {
      return CalibratedCameraT( pose_ * c.pose(), k_ ) ;
    }

    ///TODO: comment
    inline const CalibratedCameraT inverse() const {
      return CalibratedCameraT( pose_.inverse(), k_ ) ;
    }

  	/// @}
  	/// @name Testable
  	/// @{

    /// assert equality up to a tolerance
    bool equals (const CalibratedCameraT &camera, double tol = 1e-9) const {
      return pose_.equals(camera.pose(), tol) && k_.equals(camera.calibration(), tol) ;
    }

    /// print
    void print(const std::string& s = "") const {
      pose_.print("pose3");
      k_.print("calibration");
    }

  	/// @}
  	/// @name Manifold
  	/// @{

    ///TODO: comment
    CalibratedCameraT retract(const Vector& d) const {
      return CalibratedCameraT(pose().retract(d), k_) ;
    }

    ///TODO: comment
    Vector localCoordinates(const CalibratedCameraT& T2) const {
      return pose().localCoordinates(T2.pose()) ;
    }

    ///TODO: comment
    inline size_t dim() const { return 6 ; }	//TODO: add final dimension variable?

    ///TODO: comment
    inline static size_t Dim() { return 6 ; }	//TODO: add final dimension variable?

    //TODO: remove comment and method?
    /**
     * Create a level camera at the given 2D pose and height
     * @param pose2 specifies the location and viewing direction
     * (theta 0 = looking in direction of positive X axis)
     */
    // static CalibratedCameraT level(const Pose2& pose2, double height);

    /* ************************************************************************* */
    // measurement functions and derivatives
    /* ************************************************************************* */

    /**
     * This function receives the camera pose and the landmark location and
     * returns the location the point is supposed to appear in the image
     * @param camera the CalibratedCameraT
     * @param point a 3D point to be projected
     * @return the intrinsic coordinates of the projected point
     */

    /// @}
    /// @name Transformations
    /// @{

    ///TODO: comment
    inline Point2 project(const Point3& point,
      boost::optional<Matrix&> D_intrinsic_pose = boost::none,
      boost::optional<Matrix&> D_intrinsic_point = boost::none) const {
      std::pair<Point2,bool> result = projectSafe(point, D_intrinsic_pose, D_intrinsic_point) ;
      return result.first ;
    }

    ///TODO: comment
    std::pair<Point2,bool> projectSafe(
        const Point3& pw,
        boost::optional<Matrix&> D_intrinsic_pose = boost::none,
        boost::optional<Matrix&> D_intrinsic_point = boost::none) const {

      if ( !D_intrinsic_pose && !D_intrinsic_point ) {
        Point3 pc = pose_.transform_to(pw) ;
        Point2 pn = project_to_camera(pc) ;
        return std::make_pair(k_.uncalibrate(pn), pc.z() > 0) ;
      }
      // world to camera coordinate
      Matrix Hc1 /* 3*6 */, Hc2 /* 3*3 */ ;
      Point3 pc = pose_.transform_to(pw, Hc1, Hc2) ;
      // camera to normalized image coordinate
      Matrix Hn; // 2*3
      Point2 pn = project_to_camera(pc, Hn) ;
      // uncalibration
      Matrix Hi; // 2*2
      Point2 pi = k_.uncalibrate(pn,boost::none,Hi);
      Matrix tmp = Hi*Hn ;
      if (D_intrinsic_pose) *D_intrinsic_pose = tmp * Hc1 ;
      if (D_intrinsic_point) *D_intrinsic_point = tmp * Hc2 ;
      return std::make_pair(pi, pc.z()>0) ;
    }

    /**
    * projects a 3-dimensional point in camera coordinates into the
    * camera and returns a 2-dimensional point, no calibration applied
    * With optional 2by3 derivative
    */
    static Point2 project_to_camera(const Point3& P,
                             boost::optional<Matrix&> H1 = boost::none){
      if (H1) {
        double d = 1.0 / P.z(), d2 = d * d;
        *H1 = Matrix_(2, 3, d, 0.0, -P.x() * d2, 0.0, d, -P.y() * d2);
      }
      return Point2(P.x() / P.z(), P.y() / P.z());
    }

    /**
     * backproject a 2-dimensional point to a 3-dimension point
     */
    Point3 backproject_from_camera(const Point2& pi, const double scale) const {
      Point2 pn = k_.calibrate(pi);
      Point3 pc(pn.x()*scale, pn.y()*scale, scale);
      return pose_.transform_from(pc);
    }

private:

    /// @}
    /// @name Advanced Interface
    /// @{

      /** Serialization function */
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(pose_);
        ar & BOOST_SERIALIZATION_NVP(k_);
      }

      /// @}
  };
}

//TODO: remove?

//    static CalibratedCameraT Expmap(const Vector& v) {
//      return CalibratedCameraT(Pose3::Expmap(v), k_) ;
//    }
//    static Vector Logmap(const CalibratedCameraT& p) {
//      return Pose3::Logmap(p.pose()) ;
//    }

//    Point2 project(const Point3& point,
//      boost::optional<Matrix&> D_intrinsic_pose = boost::none,
//      boost::optional<Matrix&> D_intrinsic_point = boost::none) const {
//
//      // no derivative is necessary
//      if ( !D_intrinsic_pose && !D_intrinsic_point ) {
//        Point3 pc = pose_.transform_to(point) ;
//        Point2 pn = project_to_camera(pc) ;
//        return k_.uncalibrate(pn) ;
//      }
//
//      // world to camera coordinate
//      Matrix Hc1 /* 3*6 */, Hc2 /* 3*3 */ ;
//      Point3 pc = pose_.transform_to(point, Hc1, Hc2) ;
//
//      // camera to normalized image coordinate
//      Matrix Hn; // 2*3
//      Point2 pn = project_to_camera(pc, Hn) ;
//
//      // uncalibration
//      Matrix Hi; // 2*2
//      Point2 pi = k_.uncalibrate(pn,boost::none,Hi);
//
//      Matrix tmp = Hi*Hn ;
//      *D_intrinsic_pose = tmp * Hc1 ;
//      *D_intrinsic_point = tmp * Hc2 ;
//      return pi ;
//    }
//
//    std::pair<Point2,bool> projectSafe(
//        const Point3& pw,
//        boost::optional<Matrix&> H1 = boost::none,
//        boost::optional<Matrix&> H2 = boost::none) const {
//        Point3 pc = pose_.transform_to(pw);
//      return std::pair<Point2, bool>( project(pw,H1,H2), pc.z() > 0);
//    }
//
//    std::pair<Point2,bool> projectSafe(
//        const Point3& pw,
//        const Point3& pw_normal,
//        boost::optional<Matrix&> H1 = boost::none,
//        boost::optional<Matrix&> H2 = boost::none) const {
//      Point3 pc = pose_.transform_to(pw);
//      Point3 pc_normal = pose_.rotation().unrotate(pw_normal);
//      return std::pair<Point2, bool>( project(pw,H1,H2), (pc.z() > 0) && (pc_normal.z() < -0.5) );
//    }

