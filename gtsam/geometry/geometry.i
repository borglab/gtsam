//*************************************************************************
// geometry
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Point2.h>
class Point2 {
  // Standard Constructors
  Point2();
  Point2(double x, double y);
  Point2(Vector v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Point2& point, double tol) const;

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

  // enable pickling in python
  void pickle() const;
};
  
class Point2Pairs {
  Point2Pairs();
  size_t size() const;
  bool empty() const;
  gtsam::Point2Pair at(size_t n) const;
  void push_back(const gtsam::Point2Pair& point_pair);
};

// std::vector<gtsam::Point2>
class Point2Vector {
  // Constructors
  Point2Vector();
  Point2Vector(const gtsam::Point2Vector& v);

  // Capacity
  size_t size() const;
  size_t max_size() const;
  void resize(size_t sz);
  size_t capacity() const;
  bool empty() const;
  void reserve(size_t n);

  // Element access
  gtsam::Point2 at(size_t n) const;
  gtsam::Point2 front() const;
  gtsam::Point2 back() const;

  // Modifiers
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
  void print(string s = "") const;
  bool equals(const gtsam::StereoPoint2& point, double tol) const;

  // Group
  static gtsam::StereoPoint2 identity();
  gtsam::StereoPoint2 inverse() const;
  gtsam::StereoPoint2 compose(const gtsam::StereoPoint2& p2) const;
  gtsam::StereoPoint2 between(const gtsam::StereoPoint2& p2) const;

  // Operator Overloads
  gtsam::StereoPoint2 operator-() const;
  // gtsam::StereoPoint2 operator+(Vector b) const;  //TODO Mixed types not yet
  // supported
  gtsam::StereoPoint2 operator+(const gtsam::StereoPoint2& p2) const;
  gtsam::StereoPoint2 operator-(const gtsam::StereoPoint2& p2) const;

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

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Point3.h>
class Point3 {
  // Standard Constructors
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);

  // Testable
  void print(string s = "") const;
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

  // enable pickling in python
  void pickle() const;
};

class Point3Pairs {
  Point3Pairs();
  size_t size() const;
  bool empty() const;
  gtsam::Point3Pair at(size_t n) const;
  void push_back(const gtsam::Point3Pair& point_pair);
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
  void print(string s = "theta") const;
  bool equals(const gtsam::Rot2& rot, double tol) const;

  // Group
  static gtsam::Rot2 identity();
  gtsam::Rot2 inverse();
  gtsam::Rot2 compose(const gtsam::Rot2& p2) const;
  gtsam::Rot2 between(const gtsam::Rot2& p2) const;

  // Operator Overloads
  gtsam::Rot2 operator*(const gtsam::Rot2& p2) const;

  // Manifold
  gtsam::Rot2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot2& p) const;

  // Lie Group
  static gtsam::Rot2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot2& p);
  Vector logmap(const gtsam::Rot2& p);

  // Group Action on Point2
  gtsam::Point2 rotate(const gtsam::Point2& point) const;
  gtsam::Point2 unrotate(const gtsam::Point2& point) const;

  // Standard Interface
  static gtsam::Rot2 relativeBearing(
      const gtsam::Point2& d);  // Ignoring derivative
  static gtsam::Rot2 atan2(double y, double x);
  double theta() const;
  double degrees() const;
  double c() const;
  double s() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/SO3.h>
class SO3 {
  // Standard Constructors
  SO3();
  SO3(Matrix R);
  static gtsam::SO3 FromMatrix(Matrix R);
  static gtsam::SO3 AxisAngle(const Vector axis, double theta);
  static gtsam::SO3 ClosestTo(const Matrix M);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SO3& other, double tol) const;

  // Group
  static gtsam::SO3 identity();
  gtsam::SO3 inverse() const;
  gtsam::SO3 between(const gtsam::SO3& R) const;
  gtsam::SO3 compose(const gtsam::SO3& R) const;

  // Operator Overloads
  gtsam::SO3 operator*(const gtsam::SO3& R) const;

  // Manifold
  gtsam::SO3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::SO3& R) const;
  static gtsam::SO3 Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;
};

#include <gtsam/geometry/SO4.h>
class SO4 {
  // Standard Constructors
  SO4();
  SO4(Matrix R);
  static gtsam::SO4 FromMatrix(Matrix R);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SO4& other, double tol) const;

  // Group
  static gtsam::SO4 identity();
  gtsam::SO4 inverse() const;
  gtsam::SO4 between(const gtsam::SO4& Q) const;
  gtsam::SO4 compose(const gtsam::SO4& Q) const;

  // Operator Overloads
  gtsam::SO4 operator*(const gtsam::SO4& Q) const;

  // Manifold
  gtsam::SO4 retract(Vector v) const;
  Vector localCoordinates(const gtsam::SO4& Q) const;
  static gtsam::SO4 Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;
};

#include <gtsam/geometry/SOn.h>
class SOn {
  // Standard Constructors
  SOn(size_t n);
  static gtsam::SOn FromMatrix(Matrix R);
  static gtsam::SOn Lift(size_t n, Matrix R);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SOn& other, double tol) const;

  // Group
  static gtsam::SOn identity();
  gtsam::SOn inverse() const;
  gtsam::SOn between(const gtsam::SOn& Q) const;
  gtsam::SOn compose(const gtsam::SOn& Q) const;

  // Operator Overloads
  gtsam::SOn operator*(const gtsam::SOn& Q) const;

  // Manifold
  gtsam::SOn retract(Vector v) const;
  Vector localCoordinates(const gtsam::SOn& Q) const;
  static gtsam::SOn Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Quaternion.h>
class Quaternion {
  double w() const;
  double x() const;
  double y() const;
  double z() const;
  Vector coeffs() const;
};

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(Matrix R);
  Rot3(const gtsam::Point3& col1, const gtsam::Point3& col2,
       const gtsam::Point3& col3);
  Rot3(double R11, double R12, double R13, double R21, double R22, double R23,
       double R31, double R32, double R33);
  Rot3(double w, double x, double y, double z);

  static gtsam::Rot3 Rx(double t);
  static gtsam::Rot3 Ry(double t);
  static gtsam::Rot3 Rz(double t);
  static gtsam::Rot3 RzRyRx(double x, double y, double z);
  static gtsam::Rot3 RzRyRx(Vector xyz);
  static gtsam::Rot3 Yaw(
      double t);  // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 Pitch(
      double t);  // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 Roll(
      double t);  // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 Ypr(double y, double p, double r);
  static gtsam::Rot3 Quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 AxisAngle(const gtsam::Point3& axis, double angle);
  static gtsam::Rot3 Rodrigues(Vector v);
  static gtsam::Rot3 Rodrigues(double wx, double wy, double wz);
  static gtsam::Rot3 ClosestTo(const Matrix M);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Rot3& rot, double tol) const;

  // Group
  static gtsam::Rot3 identity();
  gtsam::Rot3 inverse() const;
  gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
  gtsam::Rot3 between(const gtsam::Rot3& p2) const;

  // Operator Overloads
  gtsam::Rot3 operator*(const gtsam::Rot3& p2) const;

  // Manifold
  // gtsam::Rot3 retractCayley(Vector v) const; // TODO, does not exist in both
  // Matrix and Quaternion options
  gtsam::Rot3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot3& p) const;

  // Group Action on Point3
  gtsam::Point3 rotate(const gtsam::Point3& p) const;
  gtsam::Point3 unrotate(const gtsam::Point3& p) const;

  // Standard Interface
  static gtsam::Rot3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot3& p);
  Vector logmap(const gtsam::Rot3& p);
  Matrix matrix() const;
  Matrix transpose() const;
  gtsam::Point3 column(size_t index) const;
  Vector xyz() const;
  Vector ypr() const;
  Vector rpy() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
  pair<gtsam::Unit3, double> axisAngle() const;
  gtsam::Quaternion toQuaternion() const;
  Vector quaternion() const;
  gtsam::Rot3 slerp(double t, const gtsam::Rot3& other) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Pose2.h>
class Pose2 {
  // Standard Constructor
  Pose2();
  Pose2(const gtsam::Pose2& other);
  Pose2(double x, double y, double theta);
  Pose2(double theta, const gtsam::Point2& t);
  Pose2(const gtsam::Rot2& r, const gtsam::Point2& t);
  Pose2(Vector v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose2& pose, double tol) const;

  // Group
  static gtsam::Pose2 identity();
  gtsam::Pose2 inverse() const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2) const;

  // Operator Overloads
  gtsam::Pose2 operator*(const gtsam::Pose2& p2) const;

  // Manifold
  gtsam::Pose2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose2& p) const;

  // Lie Group
  static gtsam::Pose2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose2& p);
  Vector logmap(const gtsam::Pose2& p);
  static Matrix ExpmapDerivative(Vector v);
  static Matrix LogmapDerivative(const gtsam::Pose2& v);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  static Matrix adjointMap_(Vector v);
  static Vector adjoint_(Vector xi, Vector y);
  static Vector adjointTranspose(Vector xi, Vector y);
  static Matrix wedge(double vx, double vy, double w);

  // Group Actions on Point2
  gtsam::Point2 transformFrom(const gtsam::Point2& p) const;
  gtsam::Point2 transformTo(const gtsam::Point2& p) const;

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

  // enable pickling in python
  void pickle() const;
};
  
boost::optional<gtsam::Pose2> align(const gtsam::Point2Pairs& pairs);

#include <gtsam/geometry/Pose3.h>
class Pose3 {
  // Standard Constructors
  Pose3();
  Pose3(const gtsam::Pose3& other);
  Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
  Pose3(const gtsam::Pose2& pose2);
  Pose3(Matrix mat);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose) const;

  // Operator Overloads
  gtsam::Pose3 operator*(const gtsam::Pose3& pose) const;

  // Manifold
  gtsam::Pose3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose3& pose) const;

  // Lie Group
  static gtsam::Pose3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose3& pose);
  gtsam::Pose3 expmap(Vector v);
  Vector logmap(const gtsam::Pose3& pose);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  Vector AdjointTranspose(Vector xi) const;
  static Matrix adjointMap(Vector xi);
  static Vector adjoint(Vector xi, Vector y);
  static Matrix adjointMap_(Vector xi);
  static Vector adjoint_(Vector xi, Vector y);
  static Vector adjointTranspose(Vector xi, Vector y);
  static Matrix ExpmapDerivative(Vector xi);
  static Matrix LogmapDerivative(const gtsam::Pose3& xi);
  static Matrix wedge(double wx, double wy, double wz, double vx, double vy,
                      double vz);

  // Group Action on Point3
  gtsam::Point3 transformFrom(const gtsam::Point3& point) const;
  gtsam::Point3 transformTo(const gtsam::Point3& point) const;

  // Standard Interface
  gtsam::Rot3 rotation() const;
  gtsam::Point3 translation() const;
  double x() const;
  double y() const;
  double z() const;
  Matrix matrix() const;
  gtsam::Pose3 transformPoseFrom(const gtsam::Pose3& pose) const;
  gtsam::Pose3 transformPoseTo(const gtsam::Pose3& pose) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

class Pose3Pairs {
  Pose3Pairs();
  size_t size() const;
  bool empty() const;
  gtsam::Pose3Pair at(size_t n) const;
  void push_back(const gtsam::Pose3Pair& pose_pair);
};

class Pose3Vector {
  Pose3Vector();
  size_t size() const;
  bool empty() const;
  gtsam::Pose3 at(size_t n) const;
  void push_back(const gtsam::Pose3& pose);
};

#include <gtsam/geometry/Unit3.h>
class Unit3 {
  // Standard Constructors
  Unit3();
  Unit3(const gtsam::Point3& pose);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Unit3& pose, double tol) const;

  // Other functionality
  Matrix basis() const;
  Matrix skew() const;
  gtsam::Point3 point3() const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Unit3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Unit3& s) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;

  // enabling function to compare objects
  bool equals(const gtsam::Unit3& expected, double tol) const;
};

#include <gtsam/geometry/EssentialMatrix.h>
class EssentialMatrix {
  // Standard Constructors
  EssentialMatrix(const gtsam::Rot3& aRb, const gtsam::Unit3& aTb);

  // Testable
  void print(string s = "") const;
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
  void print(string s = "Cal3_S2") const;
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
  Matrix K() const;
  Matrix inverse() const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Cal3DS2_Base.h>
virtual class Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2_Base();

  // Testable
  void print(string s = "") const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  double k1() const;
  double k2() const;
  Matrix K() const;
  Vector k() const;
  Vector vector() const;

  // Action on Point2
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Cal3DS2.h>
virtual class Cal3DS2 : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2();
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1,
          double k2);
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1,
          double k2, double p1, double p2);
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

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Cal3Unified.h>
virtual class Cal3Unified : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3Unified();
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1,
              double k2);
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1,
              double k2, double p1, double p2, double xi);
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

  // Action on Point2
  // Note: the signature of this functions differ from the functions
  // with equal name in the base class.
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Cal3Fisheye.h>
class Cal3Fisheye {
  // Standard Constructors
  Cal3Fisheye();
  Cal3Fisheye(double fx, double fy, double s, double u0, double v0, double k1,
              double k2, double k3, double k4, double tol = 1e-5);
  Cal3Fisheye(Vector v);

  // Testable
  void print(string s = "Cal3Fisheye") const;
  bool equals(const gtsam::Cal3Fisheye& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3Fisheye retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Fisheye& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double k1() const;
  double k2() const;
  double k3() const;
  double k4() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  Vector vector() const;
  Vector k() const;
  Matrix K() const;
  Matrix inverse() const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Cal3_S2Stereo.h>
class Cal3_S2Stereo {
  // Standard Constructors
  Cal3_S2Stereo();
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b);
  Cal3_S2Stereo(Vector v);

  // Testable
  void print(string s = "") const;
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
  Cal3Bundler(double fx, double k1, double k2, double u0, double v0,
              double tol);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Cal3Bundler& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3Bundler retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Bundler& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double k1() const;
  double k2() const;
  double px() const;
  double py() const;
  Vector vector() const;
  Vector k() const;
  Matrix K() const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/CalibratedCamera.h>
class CalibratedCamera {
  // Standard Constructors and Named Constructors
  CalibratedCamera();
  CalibratedCamera(const gtsam::Pose3& pose);
  CalibratedCamera(Vector v);
  static gtsam::CalibratedCamera Level(const gtsam::Pose2& pose2,
                                       double height);

  // Testable
  void print(string s = "CalibratedCamera") const;
  bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::CalibratedCamera retract(Vector d) const;
  Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

  // Action on Point3
  gtsam::Point2 project(const gtsam::Point3& point) const;
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);

  // Standard Interface
  gtsam::Pose3 pose() const;
  double range(const gtsam::Point3& point) const;
  double range(const gtsam::Pose3& pose) const;
  double range(const gtsam::CalibratedCamera& camera) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/PinholeCamera.h>
template <CALIBRATION>
class PinholeCamera {
  // Standard Constructors and Named Constructors
  PinholeCamera();
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
  This retract(Vector d) const;
  Vector localCoordinates(const This& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2, bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/Similarity3.h>
class Similarity3 {
  // Standard Constructors
  Similarity3();
  Similarity3(double s);
  Similarity3(const gtsam::Rot3& R, const gtsam::Point3& t, double s);
  Similarity3(const Matrix& R, const Vector& t, double s);
  Similarity3(const Matrix& T);

  gtsam::Point3 transformFrom(const gtsam::Point3& p) const;
  gtsam::Pose3 transformFrom(const gtsam::Pose3& T);

  static gtsam::Similarity3 Align(const gtsam::Point3Pairs& abPointPairs);
  static gtsam::Similarity3 Align(const gtsam::Pose3Pairs& abPosePairs);

  // Standard Interface
  const Matrix matrix() const;
  const gtsam::Rot3& rotation();
  const gtsam::Point3& translation();
  double scale() const;
};

// Forward declaration of PinholeCameraCalX is defined here.
#include <gtsam/geometry/SimpleCamera.h>
// Some typedefs for common camera types
// PinholeCameraCal3_S2 is the same as SimpleCamera above
typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;
typedef gtsam::PinholeCamera<gtsam::Cal3DS2> PinholeCameraCal3DS2;
typedef gtsam::PinholeCamera<gtsam::Cal3Unified> PinholeCameraCal3Unified;
typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;
typedef gtsam::PinholeCamera<gtsam::Cal3Fisheye> PinholeCameraCal3Fisheye;

template <T>
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
  gtsam::StereoCamera retract(Vector d) const;
  Vector localCoordinates(const gtsam::StereoCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  gtsam::StereoPoint2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::StereoPoint2& p) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
};

#include <gtsam/geometry/triangulation.h>

// Templates appear not yet supported for free functions - issue raised at
// borglab/wrap#14 to add support
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3_S2* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3DS2* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
                                gtsam::Cal3Bundler* sharedCal,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3_S2& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Bundler& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Fisheye& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::CameraSetCal3Unified& cameras,
                                const gtsam::Point2Vector& measurements,
                                double rank_tol, bool optimize);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3_S2* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3DS2* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::Pose3Vector& poses,
                                   gtsam::Cal3Bundler* sharedCal,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3_S2& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);
gtsam::Point3 triangulateNonlinear(const gtsam::CameraSetCal3Bundler& cameras,
                                   const gtsam::Point2Vector& measurements,
                                   const gtsam::Point3& initialEstimate);

#include <gtsam/geometry/BearingRange.h>
template <POSE, POINT, BEARING, RANGE>
class BearingRange {
  BearingRange(const BEARING& b, const RANGE& r);
  BEARING bearing() const;
  RANGE range() const;
  static This Measure(const POSE& pose, const POINT& point);
  static BEARING MeasureBearing(const POSE& pose, const POINT& point);
  static RANGE MeasureRange(const POSE& pose, const POINT& point);
  void print(string s = "") const;
};

typedef gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>
    BearingRange2D;

}  // namespace gtsam
