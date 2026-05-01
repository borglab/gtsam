//*************************************************************************
// cameras
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3.h>
virtual class Cal3 {
  // Standard Constructors
  Cal3();
  Cal3(double fx, double fy, double s, double u0, double v0);
  Cal3(gtsam::Vector v);

  // Testable
  void print(string s = "Cal3") const;
  bool equals(const gtsam::Cal3& K, double tol) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double aspectRatio() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  gtsam::Vector vector() const;
  gtsam::Matrix K() const;
  gtsam::Matrix inverse() const;
};

#include <gtsam/geometry/Cal3_S2.h>
virtual class Cal3_S2 : gtsam::Cal3 {
  // Standard Constructors
  Cal3_S2();
  Cal3_S2(double fx, double fy, double s, double u0, double v0);
  Cal3_S2(gtsam::Vector v);
  Cal3_S2(double fov, int w, int h);

  // Testable
  void print(string s = "Cal3_S2") const;
  bool equals(const gtsam::Cal3_S2& K, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3_S2 retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3_S2& T2) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;

  // Action on Homogeneous Coordinates
  gtsam::Vector3 calibrate(const gtsam::Vector3& p) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2_Base.h>
virtual class Cal3DS2_Base : gtsam::Cal3 {
  // Standard Constructors
  Cal3DS2_Base();

  // Testable
  void print(string s = "") const;

  // Standard Interface
  double k1() const;
  double k2() const;
  double p1() const;
  double p2() const;
  gtsam::Vector4 k() const;
  gtsam::Vector9 vector() const;

  // Action on Point2
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2.h>
virtual class Cal3DS2 : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2();
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1,
          double k2);
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1,
          double k2, double p1, double p2);
  Cal3DS2(gtsam::Vector v);

  // Testable
  bool equals(const gtsam::Cal3DS2& K, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3DS2 retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3DS2& T2) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3Unified.h>
virtual class Cal3Unified : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3Unified();
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1,
              double k2);
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1,
              double k2, double p1, double p2, double xi);
  Cal3Unified(gtsam::Vector v);

  // Testable
  bool equals(const gtsam::Cal3Unified& K, double tol) const;

  // Standard Interface
  double xi() const;
  gtsam::Point2 spaceToNPlane(const gtsam::Point2& p) const;
  gtsam::Point2 nPlaneToSpace(const gtsam::Point2& p) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3Unified retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3Unified& T2) const;

  // Action on Point2
  // Note: the signature of this functions differ from the functions
  // with equal name in the base class.
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3Fisheye.h>
virtual class Cal3Fisheye : gtsam::Cal3 {
  // Standard Constructors
  Cal3Fisheye();
  Cal3Fisheye(double fx, double fy, double s, double u0, double v0, double k1,
              double k2, double k3, double k4, double tol = 1e-5);
  Cal3Fisheye(gtsam::Vector v);

  // Testable
  void print(string s = "Cal3Fisheye") const;
  bool equals(const gtsam::Cal3Fisheye& K, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3Fisheye retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3Fisheye& T2) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;

  // Standard Interface
  double k1() const;
  double k2() const;
  double k3() const;
  double k4() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3_S2Stereo.h>
virtual class Cal3_S2Stereo : gtsam::Cal3_S2{
  // Standard Constructors
  Cal3_S2Stereo();
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b);
  Cal3_S2Stereo(gtsam::Vector v);
  Cal3_S2Stereo(double fov, int w, int h, double b);

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3_S2Stereo retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3_S2Stereo& T2) const;

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Cal3_S2Stereo& other, double tol) const;

  // Standard Interface
  double baseline() const;
  gtsam::Vector6 vector() const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;
};

#include <gtsam/geometry/Cal3Bundler.h>
virtual class Cal3f : gtsam::Cal3 {
  // Standard Constructors
  Cal3f();
  Cal3f(double fx, double u0, double v0);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Cal3f& K, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3f retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3f& T2) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& pi) const;
  gtsam::Point2 calibrate(const gtsam::Point2& pi,
                          Eigen::Ref<Eigen::MatrixXd> Dcal,
                          Eigen::Ref<Eigen::MatrixXd> Dp) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p,
                            Eigen::Ref<Eigen::MatrixXd> Dcal,
                            Eigen::Ref<Eigen::MatrixXd> Dp) const;

  // Standard Interface
  double f() const;

  // enabling serialization functionality
  void serialize() const;
};


#include <gtsam/geometry/Cal3Bundler.h>
virtual class Cal3Bundler : gtsam::Cal3f {
  // Standard Constructors
  Cal3Bundler();
  Cal3Bundler(double fx, double k1, double k2, double u0, double v0);
  Cal3Bundler(double fx, double k1, double k2, double u0, double v0,
              double tol);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Cal3Bundler& K, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3Bundler retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::Cal3Bundler& T2) const;

  // Standard Interface
  double k1() const;
  double k2() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/FundamentalMatrix.h>

// FundamentalMatrix class
class FundamentalMatrix {
  // Constructors
  FundamentalMatrix();
  FundamentalMatrix(const gtsam::Matrix3& U, double s, const gtsam::Matrix3& V);
  FundamentalMatrix(const gtsam::Matrix3& F);

  // Overloaded constructors for specific calibration types
  FundamentalMatrix(const gtsam::Matrix3& Ka, const gtsam::EssentialMatrix& E,
                    const gtsam::Matrix3& Kb);
  FundamentalMatrix(const gtsam::Matrix3& Ka, const gtsam::Pose3& aPb,
                    const gtsam::Matrix3& Kb);

  // Methods
  gtsam::Matrix3 matrix() const;

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::FundamentalMatrix& other, double tol = 1e-9) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Vector localCoordinates(const gtsam::FundamentalMatrix& F) const;
  gtsam::FundamentalMatrix retract(const gtsam::Vector& delta) const;
};

// SimpleFundamentalMatrix class
class SimpleFundamentalMatrix {
  // Constructors
  SimpleFundamentalMatrix();
  SimpleFundamentalMatrix(const gtsam::EssentialMatrix& E, double fa, double fb,
                          const gtsam::Point2& ca, const gtsam::Point2& cb);

  // Methods
  gtsam::Matrix3 matrix() const;

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SimpleFundamentalMatrix& other, double tol = 1e-9) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Vector localCoordinates(const gtsam::SimpleFundamentalMatrix& F) const;
  gtsam::SimpleFundamentalMatrix retract(const gtsam::Vector& delta) const;
};

gtsam::Point2 EpipolarTransfer(const gtsam::Matrix3& Fca, const gtsam::Point2& pa,
                               const gtsam::Matrix3& Fcb, const gtsam::Point2& pb);

#include <gtsam/geometry/CalibratedCamera.h>
class CalibratedCamera {
  // Standard Constructors and Named Constructors
  CalibratedCamera();
  CalibratedCamera(const gtsam::Pose3& pose);
  CalibratedCamera(gtsam::Vector v);
  static gtsam::CalibratedCamera Level(const gtsam::Pose2& pose2,
                                       double height);

  // Testable
  void print(string s = "CalibratedCamera") const;
  bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::CalibratedCamera retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

  // Action on Point3
  gtsam::Point2 project(const gtsam::Point3& point) const;
  gtsam::Point2 project(const gtsam::Point3& point,
                        Eigen::Ref<Eigen::MatrixXd> Dcamera,
                        Eigen::Ref<Eigen::MatrixXd> Dpoint);
  gtsam::Point3 backproject(const gtsam::Point2& pn, double depth) const;
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dpose,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dp,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth);

  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);

  // Standard Interface
  gtsam::Pose3 pose() const;
  double range(const gtsam::Point3& point) const;
  double range(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpoint);
  double range(const gtsam::Pose3& pose) const;
  double range(const gtsam::Pose3& point, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpose);
  double range(const gtsam::CalibratedCamera& camera) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/PinholeCamera.h>
template <CALIBRATION>
class PinholeCamera {
  // Standard Constructors and Named Constructors
  PinholeCamera();
  PinholeCamera(const This other);
  PinholeCamera(const gtsam::Pose3& pose);
  PinholeCamera(const gtsam::Pose3& pose, const CALIBRATION& K);
  static This Level(const CALIBRATION& K, const gtsam::Pose2& pose,
                    double height);
  static This Level(const gtsam::Pose2& pose, double height);
  static This Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
                     const gtsam::Point3& upVector, const CALIBRATION& K);

  // Testable
  void print(string s = "PinholeCamera") const;
  bool equals(const This& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  CALIBRATION calibration() const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  This retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const This& T2) const;

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2, bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point2 project(const gtsam::Point3& point,
    Eigen::Ref<Eigen::MatrixXd> Dpose);
  gtsam::Point2 project(const gtsam::Point3& point,
    Eigen::Ref<Eigen::MatrixXd> Dpose,
    Eigen::Ref<Eigen::MatrixXd> Dpoint);
  gtsam::Point2 project(const gtsam::Point3& point,
    Eigen::Ref<Eigen::MatrixXd> Dpose,
    Eigen::Ref<Eigen::MatrixXd> Dpoint,
    Eigen::Ref<Eigen::MatrixXd> Dcal);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dpose,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dp,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dcal);

  gtsam::Point2 reprojectionError(const gtsam::Point3& pw, const gtsam::Point2& measured,
                                  Eigen::Ref<Eigen::MatrixXd> Dpose,
                                  Eigen::Ref<Eigen::MatrixXd> Dpoint,
                                  Eigen::Ref<Eigen::MatrixXd> Dcal);

  double range(const gtsam::Point3& point);
  double range(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpoint);
  double range(const gtsam::Pose3& pose);
  double range(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpose);

  // enabling serialization functionality
  void serialize() const;
};

// Forward declaration of PinholeCameraCalX is defined here.
#include <gtsam/geometry/SimpleCamera.h>
// Some typedefs for common camera types
// PinholeCameraCal3_S2 is the same as SimpleCamera above
typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;
typedef gtsam::PinholeCamera<gtsam::Cal3DS2> PinholeCameraCal3DS2;
typedef gtsam::PinholeCamera<gtsam::Cal3Unified> PinholeCameraCal3Unified;
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;
typedef gtsam::PinholeCamera<gtsam::Cal3f> PinholeCameraCal3f;
typedef gtsam::PinholeCamera<gtsam::Cal3Fisheye> PinholeCameraCal3Fisheye;

#include <gtsam/geometry/PinholePose.h>
template <CALIBRATION>
class PinholePose {
  // Standard Constructors and Named Constructors
  PinholePose();
  PinholePose(const This other);
  PinholePose(const gtsam::Pose3& pose);
  PinholePose(const gtsam::Pose3& pose, const CALIBRATION* K);
  static This Level(const gtsam::Pose2& pose, double height);
  static This Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
                     const gtsam::Point3& upVector, const CALIBRATION* K);

  // Testable
  void print(string s = "PinholePose") const;
  bool equals(const This& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  CALIBRATION calibration() const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  This retract(gtsam::Vector d) const;
  gtsam::Vector localCoordinates(const This& p) const;

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2, bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point2 project(const gtsam::Point3& point,
                        Eigen::Ref<Eigen::MatrixXd> Dpose,
                        Eigen::Ref<Eigen::MatrixXd> Dpoint,
                        Eigen::Ref<Eigen::MatrixXd> Dcal);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dpose,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dp,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth,
                            Eigen::Ref<Eigen::MatrixXd> Dresult_dcal);
  double range(const gtsam::Point3& point);
  double range(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpoint);
  double range(const gtsam::Pose3& pose);
  double range(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Dcamera,
               Eigen::Ref<Eigen::MatrixXd> Dpose);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::PinholePose<gtsam::Cal3_S2> PinholePoseCal3_S2;
typedef gtsam::PinholePose<gtsam::Cal3DS2> PinholePoseCal3DS2;
typedef gtsam::PinholePose<gtsam::Cal3Unified> PinholePoseCal3Unified;
typedef gtsam::PinholePose<gtsam::Cal3Bundler> PinholePoseCal3Bundler;
typedef gtsam::PinholePose<gtsam::Cal3Fisheye> PinholePoseCal3Fisheye;

#include <gtsam/geometry/SphericalCamera.h>
class SphericalCamera {
  // Standard Constructors
  SphericalCamera();
  SphericalCamera(const gtsam::Pose3& pose);
  SphericalCamera(const gtsam::Pose3& pose,
                  const gtsam::EmptyCal::shared_ptr& cal);
  SphericalCamera(const gtsam::Vector& v);

  // Testable
  bool equals(const gtsam::SphericalCamera& camera, double tol = 1e-9) const;
  void print(const std::string& s = "SphericalCamera") const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  const gtsam::Rot3& rotation() const;
  const gtsam::Point3& translation() const;

  // Transformations and measurement functions
  pair<gtsam::Unit3, bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Unit3 project(const gtsam::Point3& point) const;
  gtsam::Unit3 project2(const gtsam::Point3& point) const;
  gtsam::Unit3 project2(const gtsam::Unit3& pwu) const;
  gtsam::Point3 backproject(const gtsam::Unit3& p, double depth) const;
  gtsam::Unit3 backprojectPointAtInfinity(const gtsam::Unit3& p) const;
  gtsam::Vector2 reprojectionError(const gtsam::Point3& point,
                                   const gtsam::Unit3& measured) const;

  // Manifold
  gtsam::SphericalCamera retract(const gtsam::Vector6& d) const;
  gtsam::Vector6 localCoordinates(const gtsam::SphericalCamera& p) const;
  static gtsam::SphericalCamera Identity();

  // enabling serialization functionality
  void serialize() const;
};

template <T = {gtsam::PinholePoseCal3_S2}>
class CameraSet {
  CameraSet();

  // structure specific methods
  T at(size_t i) const;
  void push_back(const T& cam);
};

#include <gtsam/geometry/StereoCamera.h>
class StereoCamera {
  // Standard Constructors and Named Constructors
  StereoCamera();
  StereoCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2Stereo* K);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::StereoCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  double baseline() const;
  gtsam::Cal3_S2Stereo calibration() const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::StereoCamera retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::StereoCamera& t2) const;

  // Transformations and measurement functions
  gtsam::StereoPoint2 project(const gtsam::Point3& point) const;
  gtsam::Point3 backproject(const gtsam::StereoPoint2& z) const;

  // project with Jacobian
  gtsam::StereoPoint2 project2(const gtsam::Point3& point,
                              Eigen::Ref<Eigen::MatrixXd> H1,
                              Eigen::Ref<Eigen::MatrixXd> H2) const;

  gtsam::Point3 backproject2(const gtsam::StereoPoint2& p,
                             Eigen::Ref<Eigen::MatrixXd> H1,
                             Eigen::Ref<Eigen::MatrixXd> H2) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/triangulation.h>
class TriangulationResult {
  enum Status { VALID, DEGENERATE, BEHIND_CAMERA, OUTLIER, FAR_POINT };
  gtsam::TriangulationResult::Status status;
  TriangulationResult(const gtsam::Point3& p);
  const gtsam::Point3& get() const;
  static gtsam::TriangulationResult Degenerate();
  static gtsam::TriangulationResult Outlier();
  static gtsam::TriangulationResult FarPoint();
  static gtsam::TriangulationResult BehindCamera();
  bool valid() const;
  bool degenerate() const;
  bool outlier() const;
  bool farPoint() const;
  bool behindCamera() const;
};

class TriangulationParameters {
  double rankTolerance;
  bool enableEPI;
  double landmarkDistanceThreshold;
  double dynamicOutlierRejectionThreshold;
  bool useLOST;
  gtsam::SharedNoiseModel noiseModel;
  TriangulationParameters(const double rankTolerance = 1.0,
                          const bool enableEPI = false,
                          double landmarkDistanceThreshold = -1,
                          double dynamicOutlierRejectionThreshold = -1,
                          const bool useLOST = false,
                          const gtsam::SharedNoiseModel& noiseModel = nullptr);
};

// Can be templated but overloaded for convenience.
// We can now call `triangulatePoint3` with any template type.

// Cal3_S2 versions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3_S2* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3_S2& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr,
                                const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3_S2* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3_S2& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSetCal3_S2& cameras,
    const gtsam::Point2Vector& measurements,
    const gtsam::TriangulationParameters& params);

// Cal3DS2 versions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3DS2* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3DS2& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr,
                                const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3DS2* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3DS2& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSetCal3DS2& cameras,
    const gtsam::Point2Vector& measurements,
    const gtsam::TriangulationParameters& params);

// Cal3Bundler versions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3Bundler* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Bundler& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr,
                                const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3Bundler* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3Bundler& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSetCal3Bundler& cameras,
    const gtsam::Point2Vector& measurements,
    const gtsam::TriangulationParameters& params);

// Cal3Fisheye versions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3Fisheye* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Fisheye& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr,
                                const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3Fisheye* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3Fisheye& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSetCal3Fisheye& cameras,
    const gtsam::Point2Vector& measurements,
    const gtsam::TriangulationParameters& params);

// Cal3Unified versions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3Unified* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Unified& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize,
                                const gtsam::SharedNoiseModel& model = nullptr,
                                const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3Unified* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3Unified& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSetCal3Unified& cameras,
    const gtsam::Point2Vector& measurements,
    const gtsam::TriangulationParameters& params);

// Spherical versions
gtsam::Point3 triangulatePoint3(
    const gtsam::CameraSet<gtsam::SphericalCamera>& cameras,
    const gtsam::SphericalCamera::MeasurementVector& measurements,
    double rank_tol, bool optimize,
    const gtsam::SharedNoiseModel& model = nullptr,
    const bool useLOST = false);
gtsam::Point3 triangulateNonlinear(
    const gtsam::CameraSet<gtsam::SphericalCamera>& cameras,
    const gtsam::SphericalCamera::MeasurementVector& measurements,
    const gtsam::Point3& initialEstimate);
gtsam::TriangulationResult triangulateSafe(
    const gtsam::CameraSet<gtsam::SphericalCamera>& cameras,
    const gtsam::SphericalCamera::MeasurementVector& measurements,
    const gtsam::TriangulationParameters& params);

}  // namespace gtsam
