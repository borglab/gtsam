namespace gtsam {

#include <gtsam/base/FastVector.h>
template<T> class FastVector{};
typedef gtsam::FastVector<size_t> KeyVector;

#include <gtsam/base/FastList.h>
template<T> class FastList{};
typedef gtsam::FastList<size_t> KeyList;

#include <gtsam/base/FastSet.h>
template<T> class FastSet{};
typedef gtsam::FastSet<size_t> KeySet;

#include <gtsam/base/FastMap.h>
template<K,V> class FastMap{};

virtual class Value {
  // No constructors because this is an abstract class

  // Testable
  void print(string s) const;

  // Manifold
  size_t dim() const;
};

#include <gtsam/base/deprecated/LieScalar.h>
class LieScalar {
  // Standard constructors
  LieScalar();
  LieScalar(double d);

  // Standard interface
  double value() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieScalar& expected, double tol) const;

  // Group
  static gtsam::LieScalar identity();
  gtsam::LieScalar inverse() const;
  gtsam::LieScalar compose(const gtsam::LieScalar& p) const;
  gtsam::LieScalar between(const gtsam::LieScalar& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieScalar retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieScalar& t2) const;

  // Lie group
  static gtsam::LieScalar Expmap(Vector v);
  static Vector Logmap(const gtsam::LieScalar& p);
};

#include <gtsam/base/deprecated/LieVector.h>
class LieVector {
  // Standard constructors
  LieVector();
  LieVector(Vector v);

  // Standard interface
  Vector vector() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieVector& expected, double tol) const;

  // Group
  static gtsam::LieVector identity();
  gtsam::LieVector inverse() const;
  gtsam::LieVector compose(const gtsam::LieVector& p) const;
  gtsam::LieVector between(const gtsam::LieVector& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieVector retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieVector& t2) const;

  // Lie group
  static gtsam::LieVector Expmap(Vector v);
  static Vector Logmap(const gtsam::LieVector& p);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/base/deprecated/LieMatrix.h>
class LieMatrix {
  // Standard constructors
  LieMatrix();
  LieMatrix(Matrix v);

  // Standard interface
  Matrix matrix() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieMatrix& expected, double tol) const;

  // Group
  static gtsam::LieMatrix identity();
  gtsam::LieMatrix inverse() const;
  gtsam::LieMatrix compose(const gtsam::LieMatrix& p) const;
  gtsam::LieMatrix between(const gtsam::LieMatrix& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieMatrix retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieMatrix & t2) const;

  // Lie group
  static gtsam::LieMatrix Expmap(Vector v);
  static Vector Logmap(const gtsam::LieMatrix& p);

  // enabling serialization functionality
  void serialize() const;
};

//*************************************************************************
// geometry
//*************************************************************************

#include <gtsam/geometry/Point2.h>
class Point2 {
  // Standard Constructors
  Point2();
  Point2(double x, double y);
  Point2(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point2& pose, double tol) const;

  // Group
  static gtsam::Point2 identity();

  // Standard Interface
  double x() const;
  double y() const;
  Vector vector() const;
  double distance(const gtsam::Point2& p2) const;
  double norm() const;

  // enabling serialization functionality
  void serialize() const;
};

// std::vector<gtsam::Point2>
#include <gtsam/geometry/Point2.h>
class Point2Vector
{
  // Constructors
  Point2Vector();
  Point2Vector(const gtsam::Point2Vector& v);

  //Capacity
  size_t size() const;
  size_t max_size() const;
  void resize(size_t sz);
  size_t capacity() const;
  bool empty() const;
  void reserve(size_t n);

  //Element access
  gtsam::Point2 at(size_t n) const;
  gtsam::Point2 front() const;
  gtsam::Point2 back() const;

  //Modifiers
  void assign(size_t n, const gtsam::Point2& u);
  void push_back(const gtsam::Point2& x);
  void pop_back();
};

#include <gtsam/geometry/StereoPoint2.h>
class StereoPoint2 {
  // Standard Constructors
  StereoPoint2();
  StereoPoint2(double uL, double uR, double v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::StereoPoint2& point, double tol) const;

  // Group
  static gtsam::StereoPoint2 identity();
  gtsam::StereoPoint2 inverse() const;
  gtsam::StereoPoint2 compose(const gtsam::StereoPoint2& p2) const;
  gtsam::StereoPoint2 between(const gtsam::StereoPoint2& p2) const;

  // Manifold
  gtsam::StereoPoint2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::StereoPoint2& p) const;

  // Lie Group
  static gtsam::StereoPoint2 Expmap(Vector v);
  static Vector Logmap(const gtsam::StereoPoint2& p);

  // Standard Interface
  Vector vector() const;
  double uL() const;
  double uR() const;
  double v() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Point3.h>
class Point3 {
  // Standard Constructors
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point3& p, double tol) const;

  // Group
  static gtsam::Point3 identity();

  // Standard Interface
  Vector vector() const;
  double x() const;
  double y() const;
  double z() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Rot2.h>
class Rot2 {
  // Standard Constructors and Named Constructors
  Rot2();
  Rot2(double theta);
  static gtsam::Rot2 fromAngle(double theta);
  static gtsam::Rot2 fromDegrees(double theta);
  static gtsam::Rot2 fromCosSin(double c, double s);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Rot2& rot, double tol) const;

  // Group
  static gtsam::Rot2 identity();
  gtsam::Rot2 inverse();
  gtsam::Rot2 compose(const gtsam::Rot2& p2) const;
  gtsam::Rot2 between(const gtsam::Rot2& p2) const;

  // Manifold
  gtsam::Rot2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot2& p) const;

  // Lie Group
  static gtsam::Rot2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot2& p);

  // Group Action on Point2
  gtsam::Point2 rotate(const gtsam::Point2& point) const;
  gtsam::Point2 unrotate(const gtsam::Point2& point) const;

  // Standard Interface
  static gtsam::Rot2 relativeBearing(const gtsam::Point2& d); // Ignoring derivative
  static gtsam::Rot2 atan2(double y, double x);
  double theta() const;
  double degrees() const;
  double c() const;
  double s() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(Matrix R);
  static gtsam::Rot3 Rx(double t);
  static gtsam::Rot3 Ry(double t);
  static gtsam::Rot3 Rz(double t);
  static gtsam::Rot3 RzRyRx(double x, double y, double z);
  static gtsam::Rot3 RzRyRx(Vector xyz);
  static gtsam::Rot3 Yaw(double t); // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 Pitch(double t); // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 Roll(double t); // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 Ypr(double y, double p, double r);
  static gtsam::Rot3 Quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 Rodrigues(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Rot3& rot, double tol) const;

  // Group
  static gtsam::Rot3 identity();
    gtsam::Rot3 inverse() const;
  gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
  gtsam::Rot3 between(const gtsam::Rot3& p2) const;

  // Manifold
  //gtsam::Rot3 retractCayley(Vector v) const; // FIXME, does not exist in both Matrix and Quaternion options
  gtsam::Rot3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot3& p) const;

  // Group Action on Point3
  gtsam::Point3 rotate(const gtsam::Point3& p) const;
  gtsam::Point3 unrotate(const gtsam::Point3& p) const;

  // Standard Interface
  static gtsam::Rot3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot3& p);
  Matrix matrix() const;
  Matrix transpose() const;
  gtsam::Point3 column(size_t index) const;
  Vector xyz() const;
  Vector ypr() const;
  Vector rpy() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
//  Vector toQuaternion() const;  // FIXME: Can't cast to Vector properly
  Vector quaternion() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose2.h>
class Pose2 {
  // Standard Constructor
  Pose2();
  Pose2(const gtsam::Pose2& pose);
  Pose2(double x, double y, double theta);
  Pose2(double theta, const gtsam::Point2& t);
  Pose2(const gtsam::Rot2& r, const gtsam::Point2& t);
  Pose2(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Pose2& pose, double tol) const;

  // Group
  static gtsam::Pose2 identity();
  gtsam::Pose2 inverse() const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2) const;

  // Manifold
  gtsam::Pose2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose2& p) const;

  // Lie Group
  static gtsam::Pose2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose2& p);
  Matrix AdjointMap() const;
  Vector Adjoint(const Vector& xi) const;
  static Matrix wedge(double vx, double vy, double w);

  // Group Actions on Point2
  gtsam::Point2 transform_from(const gtsam::Point2& p) const;
  gtsam::Point2 transform_to(const gtsam::Point2& p) const;

  // Standard Interface
  double x() const;
  double y() const;
  double theta() const;
  gtsam::Rot2 bearing(const gtsam::Point2& point) const;
  double range(const gtsam::Point2& point) const;
  gtsam::Point2 translation() const;
  gtsam::Rot2 rotation() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose3.h>
class Pose3 {
  // Standard Constructors
  Pose3();
  Pose3(const gtsam::Pose3& pose);
  Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
  Pose3(const gtsam::Pose2& pose2); // FIXME: shadows Pose3(Pose3 pose)
  Pose3(Matrix t);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 compose(const gtsam::Pose3& p2) const;
  gtsam::Pose3 between(const gtsam::Pose3& p2) const;

  // Manifold
  gtsam::Pose3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose3& T2) const;

  // Lie Group
  static gtsam::Pose3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose3& p);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  static Matrix wedge(double wx, double wy, double wz, double vx, double vy, double vz);

  // Group Action on Point3
  gtsam::Point3 transform_from(const gtsam::Point3& p) const;
  gtsam::Point3 transform_to(const gtsam::Point3& p) const;

  // Standard Interface
  gtsam::Rot3 rotation() const;
  gtsam::Point3 translation() const;
  double x() const;
  double y() const;
  double z() const;
  Matrix matrix() const;
  gtsam::Pose3 transform_to(const gtsam::Pose3& pose) const; // FIXME: shadows other transform_to()
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;
};

// std::vector<gtsam::Pose3>
#include <gtsam/geometry/Pose3.h>
class Pose3Vector
{
  Pose3Vector();
  size_t size() const;
  bool empty() const;
  gtsam::Pose3 at(size_t n) const;
  void push_back(const gtsam::Pose3& x);
};

#include <gtsam/geometry/Unit3.h>
class Unit3 {
  // Standard Constructors
  Unit3();
  Unit3(const gtsam::Point3& pose);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Unit3& pose, double tol) const;

  // Other functionality
  Matrix basis() const;
  Matrix skew() const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Unit3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Unit3& s) const;
};

#include <gtsam/geometry/EssentialMatrix.h>
class EssentialMatrix {
  EssentialMatrix();
  // Standard Constructors
  EssentialMatrix(const gtsam::Rot3& aRb, const gtsam::Unit3& aTb);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::EssentialMatrix& pose, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::EssentialMatrix retract(Vector v) const;
  Vector localCoordinates(const gtsam::EssentialMatrix& s) const;

  // Other methods:
  gtsam::Rot3 rotation() const;
  gtsam::Unit3 direction() const;
  Matrix matrix() const;
  double error(Vector vA, Vector vB);
};

#include <gtsam/geometry/Cal3_S2.h>
class Cal3_S2 {
  // Standard Constructors
  Cal3_S2();
  Cal3_S2(double fx, double fy, double s, double u0, double v0);
  Cal3_S2(Vector v);
  Cal3_S2(double fov, int w, int h);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3_S2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3_S2& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  Vector vector() const;
  Matrix matrix() const;
  Matrix matrix_inverse() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2_Base.h>
virtual class Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2_Base();

  // Testable
  void print(string s) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  double k1() const;
  double k2() const;

  // Action on Point2
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p, double tol) const;
  // gtsam::Point2 calibrate(const gtsam::Point2& p) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2.h>
virtual class Cal3DS2 : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2();
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1, double k2);
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1, double k2, double p1, double p2);
  Cal3DS2(Vector v);

  // Testable
  bool equals(const gtsam::Cal3DS2& rhs, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3DS2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3DS2& c) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3Unified.h>
virtual class Cal3Unified : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3Unified();
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1, double k2);
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1, double k2, double p1, double p2, double xi);
  Cal3Unified(Vector v);

  // Testable
  bool equals(const gtsam::Cal3Unified& rhs, double tol) const;

  // Standard Interface
  double xi() const;
  gtsam::Point2 spaceToNPlane(const gtsam::Point2& p) const;
  gtsam::Point2 nPlaneToSpace(const gtsam::Point2& p) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3Unified retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Unified& c) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3_S2Stereo.h>
class Cal3_S2Stereo {
  // Standard Constructors
  Cal3_S2Stereo();
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b);
  Cal3_S2Stereo(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2Stereo& K, double tol) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  double baseline() const;
};

#include <gtsam/geometry/Cal3Bundler.h>
class Cal3Bundler {
  // Standard Constructors
  Cal3Bundler();
  Cal3Bundler(double fx, double k1, double k2, double u0, double v0);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3Bundler& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3Bundler retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Bundler& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p, double tol) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double k1() const;
  double k2() const;
  double u0() const;
  double v0() const;
  Vector vector() const;
  Vector k() const;
  //Matrix K() const; //FIXME: Uppercase

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/CalibratedCamera.h>
class CalibratedCamera {
  // Standard Constructors and Named Constructors
  CalibratedCamera();
  CalibratedCamera(const gtsam::Pose3& pose);
  CalibratedCamera(const Vector& v);
  static gtsam::CalibratedCamera Level(const gtsam::Pose2& pose2, double height);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::CalibratedCamera retract(const Vector& d) const;
  Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

  // Action on Point3
  gtsam::Point2 project(const gtsam::Point3& point) const;
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);

  // Standard Interface
  gtsam::Pose3 pose() const;
  double range(const gtsam::Point3& p) const; // TODO: Other overloaded range methods

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/PinholeCamera.h>
template<CALIBRATION>
class PinholeCamera {
  // Standard Constructors and Named Constructors
  PinholeCamera();
  PinholeCamera(const gtsam::Pose3& pose);
  PinholeCamera(const gtsam::Pose3& pose, const CALIBRATION& K);
  static This Level(const CALIBRATION& K, const gtsam::Pose2& pose, double height);
  static This Level(const gtsam::Pose2& pose, double height);
  static This Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
      const gtsam::Point3& upVector, const CALIBRATION& K);

  // Testable
  void print(string s) const;
  bool equals(const This& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  CALIBRATION calibration() const;

  // Manifold
  This retract(const Vector& d) const;
  Vector localCoordinates(const This& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2,bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& point);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/SimpleCamera.h>
virtual class SimpleCamera {
  // Standard Constructors and Named Constructors
  SimpleCamera();
  SimpleCamera(const gtsam::Pose3& pose);
  SimpleCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2& K);
  static gtsam::SimpleCamera Level(const gtsam::Cal3_S2& K, const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera Level(const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
      const gtsam::Point3& upVector, const gtsam::Cal3_S2& K);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SimpleCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  gtsam::Cal3_S2 calibration() const;

  // Manifold
  gtsam::SimpleCamera retract(const Vector& d) const;
  Vector localCoordinates(const gtsam::SimpleCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2,bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& point);

  // enabling serialization functionality
  void serialize() const;

};

// Some typedefs for common camera types
// PinholeCameraCal3_S2 is the same as SimpleCamera above
typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;
typedef gtsam::PinholeCamera<gtsam::Cal3DS2> PinholeCameraCal3DS2;
typedef gtsam::PinholeCamera<gtsam::Cal3Unified> PinholeCameraCal3Unified;
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;

#include <gtsam/geometry/StereoCamera.h>
class StereoCamera {
  // Standard Constructors and Named Constructors
  StereoCamera();
  StereoCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2Stereo* K);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::StereoCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  double baseline() const;
  gtsam::Cal3_S2Stereo calibration() const;

  // Manifold
  gtsam::StereoCamera retract(const Vector& d) const;
  Vector localCoordinates(const gtsam::StereoCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  gtsam::StereoPoint2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::StereoPoint2& p) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/triangulation.h>

// Templates appear not yet supported for free functions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
    gtsam::Cal3_S2* sharedCal, const gtsam::Point2Vector& measurements,
    double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
    gtsam::Cal3Bundler* sharedCal, const gtsam::Point2Vector& measurements,
    double rank_tol, bool optimize);


#include <gtsam/linear/VectorValues.h>
class VectorValues {
    //Constructors
  VectorValues();
  VectorValues(const gtsam::VectorValues& other);

  //Named Constructors
  static gtsam::VectorValues Zero(const gtsam::VectorValues& model);

  //Standard Interface
  size_t size() const;
  size_t dim(size_t j) const;
  bool exists(size_t j) const;
  void print(string s) const;
  bool equals(const gtsam::VectorValues& expected, double tol) const;
  void insert(size_t j, Vector value);
  Vector vector() const;
  Vector at(size_t j) const;
  void update(const gtsam::VectorValues& values);

  //Advanced Interface
  void setZero();

  gtsam::VectorValues add(const gtsam::VectorValues& c) const;
  void addInPlace(const gtsam::VectorValues& c);
  gtsam::VectorValues subtract(const gtsam::VectorValues& c) const;
  gtsam::VectorValues scale(double a) const;
  void scaleInPlace(double a);

  bool hasSameStructure(const gtsam::VectorValues& other)  const;
  double dot(const gtsam::VectorValues& V) const;
  double norm() const;
  double squaredNorm() const;

  // enabling serialization functionality
  void serialize() const;
};

namespace noiseModel {
#include <gtsam/linear/NoiseModel.h>
virtual class Base {
};

virtual class Gaussian : gtsam::noiseModel::Base {
  static gtsam::noiseModel::Gaussian* SqrtInformation(Matrix R);
  static gtsam::noiseModel::Gaussian* Covariance(Matrix R);
  Matrix R() const;
  bool equals(gtsam::noiseModel::Base& expected, double tol);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Diagonal : gtsam::noiseModel::Gaussian {
  static gtsam::noiseModel::Diagonal* Sigmas(Vector sigmas);
  static gtsam::noiseModel::Diagonal* Variances(Vector variances);
  static gtsam::noiseModel::Diagonal* Precisions(Vector precisions);

  // enabling serialization functionality
  void serializable() const;
};

namespace mEstimator {
virtual class Base {
};

virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;
};

virtual class Fair: gtsam::noiseModel::mEstimator::Base {
  Fair(double c);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Fair* Create(double c);

  // enabling serialization functionality
  void serializable() const;
};

virtual class Huber: gtsam::noiseModel::mEstimator::Base {
  Huber(double k);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Huber* Create(double k);

  // enabling serialization functionality
  void serializable() const;
};
} // namespace mEstimator

virtual class Robust : gtsam::noiseModel::Base {
  Robust(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  static gtsam::noiseModel::Robust* Create(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};
} // namespace noiseModel

#include <gtsam/linear/GaussianFactor.h>
virtual class GaussianFactor {
  // gtsam::KeyVector keys() const;
  void print(string s) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;
  gtsam::GaussianFactor* clone() const;
  gtsam::GaussianFactor* negate() const;
  Matrix augmentedInformation() const;
  Matrix information() const;
  Matrix augmentedJacobian() const;
  pair<Matrix, Vector> jacobian() const;
  size_t size() const;
  bool empty() const;
};

#include <gtsam/linear/JacobianFactor.h>
virtual class JacobianFactor : gtsam::GaussianFactor {
  //Constructors
  JacobianFactor();
  JacobianFactor(const gtsam::GaussianFactor& factor);
  JacobianFactor(Vector b_in);
  JacobianFactor(size_t i1, Matrix A1, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3, Matrix A3,
      Vector b, const gtsam::noiseModel::Diagonal* model);
  // JacobianFactor(const gtsam::GaussianFactorGraph& graph);

  //Testable
  void printKeys(string s) const;
  Vector unweighted_error(const gtsam::VectorValues& c) const;
  Vector error_vector(const gtsam::VectorValues& c) const;

  //Standard Interface
  Matrix py_getA() const;
  Vector py_getb() const;
  size_t rows() const;
  size_t cols() const;
  bool isConstrained() const;
  // pair<Matrix, Vector> jacobianUnweighted() const;
  Matrix augmentedJacobianUnweighted() const;

  void transposeMultiplyAdd(double alpha, const Vector& e, gtsam::VectorValues& x) const;
  gtsam::JacobianFactor whiten() const;

  // pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, const Vector& sigmas);

  gtsam::noiseModel::Diagonal* get_model() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/Values.h>
class Values {
  Values();
  Values(const gtsam::Values& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t dim() const;

  void print(string s) const;
  bool equals(const gtsam::Values& other, double tol) const;

  void insert(const gtsam::Values& values);
  void update(const gtsam::Values& values);
  void erase(size_t j);
  void swap(gtsam::Values& values);

  bool exists(size_t j) const;
  // gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // New in 4.0, we have to specialize every insert/update/at to generate wrappers
  // Instead of the old:
  // void insert(size_t j, const gtsam::Value& value);
  // void update(size_t j, const gtsam::Value& val);
  // gtsam::Value at(size_t j) const;

  template<T = {gtsam::Point3, gtsam::Rot3, Matrix, Vector}>
  void insert(size_t j, const T& t);

  template<T = {gtsam::Point3, gtsam::Rot3, Matrix, Vector}>
  void update(size_t j, const T& t);

  template<T = {gtsam::Point3, gtsam::Rot3, Vector, Matrix}>
  T at(size_t j);

  /// version for double
  void insertDouble(size_t j, double c);
  double atDouble(size_t j) const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NonlinearFactor {
  // Factor base class
  size_t size() const;
  // gtsam::KeyVector keys() const;
  void print(string s) const;
  void printKeys(string s) const;
  // NonlinearFactor
  void equals(const gtsam::NonlinearFactor& other, double tol) const;
  double error(const gtsam::Values& c) const;
  size_t dim() const;
  bool active(const gtsam::Values& c) const;
  // gtsam::GaussianFactor* linearize(const gtsam::Values& c) const;
  gtsam::NonlinearFactor* clone() const;
  // gtsam::NonlinearFactor* rekey(const gtsam::KeyVector& newKeys) const; //FIXME: Conversion from KeyVector to std::vector does not happen
};


#include <gtsam/nonlinear/NonlinearFactorGraph.h>
class NonlinearFactorGraph {
  NonlinearFactorGraph();
  NonlinearFactorGraph(const gtsam::NonlinearFactorGraph& graph);

  // FactorGraph
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactorGraph& fg, double tol) const;
  size_t size() const;
  bool empty() const;
  void remove(size_t i);
  size_t nrFactors() const;
  gtsam::NonlinearFactor* at(size_t idx) const;
  void push_back(const gtsam::NonlinearFactorGraph& factors);
  void push_back(gtsam::NonlinearFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  bool exists(size_t idx) const;
  // gtsam::KeySet keys() const;

  // NonlinearFactorGraph
  double error(const gtsam::Values& values) const;
  double probPrime(const gtsam::Values& values) const;
  // gtsam::Ordering orderingCOLAMD() const;
  // // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const std::map<gtsam::Key,int>& constraints) const;
  // gtsam::GaussianFactorGraph* linearize(const gtsam::Values& values) const;
  gtsam::NonlinearFactorGraph clone() const;

  // enabling serialization functionality
  void serialize() const;
};


virtual class NoiseModelFactor: gtsam::NonlinearFactor {
  void equals(const gtsam::NoiseModelFactor& other, double tol) const;
  gtsam::noiseModel::Base* get_noiseModel() const; // deprecated by below
  gtsam::noiseModel::Base* noiseModel() const;
  Vector unwhitenedError(const gtsam::Values& x) const;
  Vector whitenedError(const gtsam::Values& x) const;
};

#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::Point3, gtsam::Rot3}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/sam/BearingFactor.h>
template<POSE, POINT, BEARING>
virtual class BearingFactor : gtsam::NoiseModelFactor {
  BearingFactor(size_t key1, size_t key2, const BEARING& measured, const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> BearingFactor2D;


}