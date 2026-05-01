//*************************************************************************
// geometry
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Point2.h>
class Point2 {
  // Standard Constructors
  Point2();
  Point2(double x, double y);
  Point2(gtsam::Vector v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Point2& point, double tol) const;

  // Group
  static gtsam::Point2 Identity();

  // Manifold
  static size_t Dim();
  size_t dim() const;

  // Standard Interface
  double x() const;
  double y() const;
  gtsam::Vector vector() const;
  double distance(const gtsam::Point2& p2) const;
  double norm() const;

  // enabling serialization functionality
  void serialize() const;
};

// Used in Matlab wrapper
class Point2Pairs {
  Point2Pairs();
  size_t size() const;
  bool empty() const;
  gtsam::Point2Pair at(size_t n) const;
  void push_back(const gtsam::Point2Pair& point_pair);
};

// std::vector<gtsam::Point2>
// Used in Matlab wrapper
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
  StereoPoint2(const gtsam::Vector3 &v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::StereoPoint2& q, double tol) const;

  // Group
  static gtsam::StereoPoint2 Identity();

  // Operator Overloads
  gtsam::StereoPoint2 operator-() const;
  // gtsam::StereoPoint2 operator+(gtsam::Vector b) const;  //TODO Mixed types not yet
  // supported
  gtsam::StereoPoint2 operator+(const gtsam::StereoPoint2& p2) const;
  gtsam::StereoPoint2 operator-(const gtsam::StereoPoint2& p2) const;

  // Standard Interface
  gtsam::Vector vector() const;
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
  Point3(gtsam::Vector v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Point3& p, double tol) const;

  // Group
  static gtsam::Point3 Identity();

  // Manifold
  static size_t Dim();
  size_t dim() const;

  // Standard Interface
  gtsam::Vector vector() const;
  double x() const;
  double y() const;
  double z() const;

  // enabling serialization functionality
  void serialize() const;

  // Other methods
  gtsam::Point3 normalize(const gtsam::Point3 &p) const;
  gtsam::Point3 normalize(const gtsam::Point3 &p, Eigen::Ref<Eigen::MatrixXd> H) const;
};

// Used in Matlab wrapper
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
  bool equals(const gtsam::Rot2& R, double tol) const;

  // Group
  static gtsam::Rot2 Identity();
  gtsam::Rot2 inverse();
  gtsam::Rot2 compose(const gtsam::Rot2& p2) const;
  gtsam::Rot2 between(const gtsam::Rot2& p2) const;

  // Operator Overloads
  gtsam::Rot2 operator*(const gtsam::Rot2& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Rot2 retract(gtsam::Vector v) const;
  gtsam::Rot2 retract(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Vector localCoordinates(const gtsam::Rot2& p) const;
  gtsam::Vector localCoordinates(const gtsam::Rot2& p, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Lie Group
  static gtsam::Rot2 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::Rot2& r);
  gtsam::Rot2 expmap(gtsam::Vector v);
  gtsam::Vector logmap(const gtsam::Rot2& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Group Action on Point2
  gtsam::Point2 rotate(const gtsam::Point2& p) const;
  gtsam::Point2 unrotate(const gtsam::Point2& p) const;

  // Standard Interface
  static gtsam::Rot2 relativeBearing(
      const gtsam::Point2& d);  // Ignoring derivative
  static gtsam::Rot2 atan2(double y, double x);
  double theta() const;
  double degrees() const;
  double c() const;
  double s() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/SO3.h>

namespace so3 {
  class Kernel {
    gtsam::Matrix3 left() const;   // a I + b W + c WW
    gtsam::Matrix3 right() const;  // a I - b W + c WW
    gtsam::Vector3 applyLeft(const gtsam::Vector3& v) const;
    gtsam::Vector3 applyRight(const gtsam::Vector3& v) const;
    gtsam::Matrix3 frechet(const gtsam::Matrix3& X) const;
    gtsam::Matrix3 applyFrechet(const gtsam::Vector3& v) const;
  };

  class InvJKernel {
    gtsam::so3::Kernel J;  // holds the forward kernel
    gtsam::Matrix3 left() const;
    gtsam::Matrix3 right() const;
    gtsam::Vector3 applyLeft(const gtsam::Vector3& v) const;
    gtsam::Vector3 applyRight(const gtsam::Vector3& v) const;
  };

  class ExpmapFunctor {
    double theta2;
    double theta;
    gtsam::Matrix3 W;
    gtsam::Matrix3 WW;
    bool nearZero;
    double A;  // A = sin(theta) / theta
    double B;  // B = (1 - cos(theta))
    ExpmapFunctor(const gtsam::Vector3& omega);
    ExpmapFunctor(double nearZeroThresholdSq, const gtsam::Vector3& axis);
    ExpmapFunctor(const gtsam::Vector3& axis, double angle);
    gtsam::Matrix3 expmap() const;
  };

  virtual class DexpFunctor : gtsam::so3::ExpmapFunctor {
    const gtsam::Vector3 omega;

    DexpFunctor(const gtsam::Vector3& omega);
    DexpFunctor(const gtsam::Vector3& omega, double nearZeroThresholdSq, double nearPiThresholdSq);

    // Kernels
    gtsam::so3::Kernel Rodrigues() const;
    gtsam::so3::Kernel Jacobian() const;
    gtsam::so3::InvJKernel InvJacobian() const;
    gtsam::so3::Kernel Gamma() const;

    // access to (lazily evaluated) coefficients
    const double C();  // (1 - A) / theta^2
    const double D();  // (1 - A/2B) / theta2
    const double E();  // Coefficient for Gamma kernel

    // Use kernel if you need to apply
    gtsam::Matrix3 rightJacobian() const;
    gtsam::Matrix3 leftJacobian() const;
  };
}

class SO3 {
  // Standard Constructors
  SO3();
  SO3(gtsam::Matrix R);
  static gtsam::SO3 FromMatrix(gtsam::Matrix R);
  static gtsam::SO3 AxisAngle(const gtsam::Vector axis, double theta);
  static gtsam::SO3 ClosestTo(const gtsam::Matrix M);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SO3& other, double tol) const;

  // Group
  static gtsam::SO3 Identity();
  gtsam::SO3 inverse() const;
  gtsam::SO3 between(const gtsam::SO3& R) const;
  gtsam::SO3 compose(const gtsam::SO3& R) const;

  // Operator Overloads
  gtsam::SO3 operator*(const gtsam::SO3& R) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::SO3 retract(gtsam::Vector3 v) const;
  gtsam::Vector3 localCoordinates(const gtsam::SO3& R) const;

  // Lie Group
  static gtsam::SO3 Expmap(gtsam::Vector3 v);
  static gtsam::Vector3 Logmap(const gtsam::SO3& p);
  static gtsam::Matrix3 ExpmapDerivative(const gtsam::Vector3& omega);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Vector3& omega);
  gtsam::SO3 expmap(gtsam::Vector3 v);
  gtsam::Vector3 logmap(const gtsam::SO3& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);
};

#include <gtsam/geometry/SO4.h>
class SO4 {
  // Standard Constructors
  SO4();
  SO4(gtsam::Matrix R);
  static gtsam::SO4 FromMatrix(gtsam::Matrix R);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SO4& other, double tol) const;

  // Group
  static gtsam::SO4 Identity();
  gtsam::SO4 inverse() const;
  gtsam::SO4 between(const gtsam::SO4& Q) const;
  gtsam::SO4 compose(const gtsam::SO4& Q) const;

  // Operator Overloads
  gtsam::SO4 operator*(const gtsam::SO4& Q) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::SO4 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::SO4& Q) const;

  // Lie Group
  static gtsam::SO4 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::SO4& p);
  gtsam::SO4 expmap(gtsam::Vector v);
  gtsam::Vector logmap(const gtsam::SO4& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

};

#include <gtsam/geometry/SOn.h>
class SOn {
  // Standard Constructors
  SOn(size_t n);
  static gtsam::SOn FromMatrix(gtsam::Matrix R);
  static gtsam::SOn Lift(size_t n, gtsam::Matrix R);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SOn& other, double tol) const;

  // Group
  static gtsam::SOn Identity();
  gtsam::SOn inverse() const;
  gtsam::SOn between(const gtsam::SOn& Q) const;
  gtsam::SOn compose(const gtsam::SOn& Q) const;

  // Operator Overloads
  gtsam::SOn operator*(const gtsam::SOn& Q) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::SOn retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::SOn& Q) const;
  static gtsam::SOn Expmap(gtsam::Vector v);

  // Lie Group
  static gtsam::SOn Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::SOn& p);
  gtsam::SOn expmap(gtsam::Vector v);
  gtsam::Vector logmap(const gtsam::SOn& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Quaternion.h>
class Quaternion {
  double w() const;
  double x() const;
  double y() const;
  double z() const;
  gtsam::Vector coeffs() const;
};

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(gtsam::Matrix R);
  Rot3(const gtsam::Point3& col1, const gtsam::Point3& col2,
       const gtsam::Point3& col3);
  Rot3(double R11, double R12, double R13, double R21, double R22, double R23,
       double R31, double R32, double R33);
  Rot3(double w, double x, double y, double z);

  static gtsam::Rot3 Rx(double t);
  static gtsam::Rot3 Ry(double t);
  static gtsam::Rot3 Rz(double t);
  static gtsam::Rot3 RzRyRx(double x, double y, double z);
  static gtsam::Rot3 RzRyRx(gtsam::Vector xyz);
  static gtsam::Rot3 Yaw(
      double t);  // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 Pitch(
      double t);  // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 Roll(
      double t);  // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 Ypr(double y, double p, double r);
  static gtsam::Rot3 Quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 AxisAngle(const gtsam::Point3& axis, double angle);
  static gtsam::Rot3 Rodrigues(gtsam::Vector v);
  static gtsam::Rot3 Rodrigues(double wx, double wy, double wz);
  static gtsam::Rot3 ClosestTo(const gtsam::Matrix M);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Rot3& p, double tol) const;

  // Group
  static gtsam::Rot3 Identity();
  gtsam::Rot3 inverse() const;
  gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
  gtsam::Rot3 between(const gtsam::Rot3& p2) const;

  // Operator Overloads
  gtsam::Rot3 operator*(const gtsam::Rot3& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  // gtsam::Rot3 retractCayley(gtsam::Vector v) const; // TODO, does not exist in both
  // gtsam::Matrix and Quaternion options
  gtsam::Rot3 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::Rot3& p) const;

  // Lie group
  static gtsam::Rot3 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::Rot3& R);
  static gtsam::Matrix3 ExpmapDerivative(const gtsam::Vector3& omega);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Vector3& omega);
  gtsam::Rot3 expmap(const gtsam::Vector& v);
  gtsam::Vector logmap(const gtsam::Rot3& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Group Action on Point3
  gtsam::Point3 rotate(const gtsam::Point3& p) const;
  gtsam::Point3 unrotate(const gtsam::Point3& p) const;

  // Group action on Unit3
  gtsam::Unit3 rotate(const gtsam::Unit3& p) const;
  gtsam::Unit3 rotate(const gtsam::Unit3& p,
                      Eigen::Ref<Eigen::MatrixXd> HR,
                      Eigen::Ref<Eigen::MatrixXd> Hp) const;
  gtsam::Unit3 unrotate(const gtsam::Unit3& p) const;

  // Standard Interface
  gtsam::Matrix transpose() const;
  gtsam::Vector xyz() const;
  gtsam::Vector ypr() const;
  gtsam::Vector rpy() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
  pair<gtsam::Unit3, double> axisAngle() const;
  gtsam::Quaternion toQuaternion() const;
  // gtsam::Vector quaternion() const; // @deprecated, see https://github.com/borglab/gtsam/pull/1219
  gtsam::Rot3 slerp(double t, const gtsam::Rot3& other) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose2.h>
class Pose2 {
  // Standard Constructor
  Pose2();
  Pose2(const gtsam::Pose2& other);
  Pose2(double x, double y, double theta);
  Pose2(double theta, const gtsam::Point2& t);
  Pose2(const gtsam::Rot2& r, const gtsam::Point2& t);
  Pose2(gtsam::Vector v);

  static std::optional<gtsam::Pose2> Align(const gtsam::Point2Pairs& abPointPairs);
  static std::optional<gtsam::Pose2> Align(const gtsam::Matrix& a, const gtsam::Matrix& b);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose2& pose, double tol) const;

  // Group
  static gtsam::Pose2 Identity();
  gtsam::Pose2 inverse() const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2) const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Operator Overloads
  gtsam::Pose2 operator*(const gtsam::Pose2& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Pose2 retract(gtsam::Vector v) const;
  gtsam::Pose2 retract(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Vector localCoordinates(const gtsam::Pose2& p) const;
  gtsam::Vector localCoordinates(const gtsam::Pose2& p, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Lie Group
  static gtsam::Pose2 Expmap(gtsam::Vector xi);
  static gtsam::Vector Logmap(const gtsam::Pose2& p);
  static gtsam::Pose2 Expmap(gtsam::Vector xi, Eigen::Ref<Eigen::MatrixXd> H);
  static gtsam::Vector Logmap(const gtsam::Pose2& p, Eigen::Ref<Eigen::MatrixXd> H);
  gtsam::Pose2 expmap(gtsam::Vector v);
  gtsam::Pose2 expmap(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);
  gtsam::Vector logmap(const gtsam::Pose2& g);
  gtsam::Vector logmap(const gtsam::Pose2& g, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);
  static gtsam::Matrix ExpmapDerivative(gtsam::Vector v);
  static gtsam::Matrix LogmapDerivative(const gtsam::Pose2& v);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Matrix adjointMap_(gtsam::Vector xi);
  static gtsam::Vector adjoint_(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Group Actions on Point2
  gtsam::Point2 transformFrom(const gtsam::Point2& p) const;
  gtsam::Point2 transformTo(const gtsam::Point2& p) const;
  gtsam::Point2 transformFrom(const gtsam::Point2& p,
    Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Point2 transformTo(const gtsam::Point2& p,
    Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;

  // gtsam::Matrix versions
  gtsam::Matrix transformFrom(const gtsam::Matrix& points) const;
  gtsam::Matrix transformTo(const gtsam::Matrix& points) const;

  // Standard Interface
  double x() const;
  double y() const;
  double theta() const;
  gtsam::Rot2 bearing(const gtsam::Point2& point) const;
  gtsam::Rot2 bearing(const gtsam::Point2& point,
    Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  double range(const gtsam::Point2& point) const;
  double range(const gtsam::Point2& point,
    Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Point2 translation() const;
  gtsam::Point2 translation(Eigen::Ref<Eigen::MatrixXd> Hself) const;
  gtsam::Rot2 rotation() const;
  gtsam::Rot2 rotation(Eigen::Ref<Eigen::MatrixXd> Hself) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose3.h>
class Pose3 {
  // Standard Constructors
  Pose3();
  Pose3(const gtsam::Pose3& other);
  Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
  Pose3(const gtsam::Pose2& pose2);
  Pose3(gtsam::Matrix mat);

  static std::optional<gtsam::Pose3> Align(const gtsam::Point3Pairs& abPointPairs);
  static std::optional<gtsam::Pose3> Align(const gtsam::Matrix& a, const gtsam::Matrix& b);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 Identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 inverse(Eigen::Ref<Eigen::MatrixXd> H) const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose) const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose,
                       Eigen::Ref<Eigen::MatrixXd> H1,
                       Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose,
                       Eigen::Ref<Eigen::MatrixXd> H1,
                       Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Pose3 slerp(double t, const gtsam::Pose3& other) const;
  gtsam::Pose3 slerp(double t, const gtsam::Pose3& other,
                           Eigen::Ref<Eigen::MatrixXd> Hx,
                           Eigen::Ref<Eigen::MatrixXd> Hy) const;

  // Operator Overloads
  gtsam::Pose3 operator*(const gtsam::Pose3& pose) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Pose3 retract(gtsam::Vector v) const;
  gtsam::Pose3 retract(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> Hxi) const;
  gtsam::Vector localCoordinates(const gtsam::Pose3& pose) const;
  gtsam::Vector localCoordinates(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hxi) const;

  // Lie Group
  static gtsam::Pose3 Expmap(gtsam::Vector xi);
  static gtsam::Vector Logmap(const gtsam::Pose3& p);
  static gtsam::Matrix6 ExpmapDerivative(const gtsam::Vector6& xi);
  static gtsam::Matrix6 LogmapDerivative(const gtsam::Vector6& xi);
  static gtsam::Matrix6 LogmapDerivative(const gtsam::Pose3& xi);
  static gtsam::Pose3 Expmap(gtsam::Vector xi, Eigen::Ref<Eigen::MatrixXd> Hxi);
  static gtsam::Vector Logmap(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hpose);

  gtsam::Pose3 expmap(gtsam::Vector v);
  gtsam::Pose3 expmap(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);
  
  gtsam::Vector logmap(const gtsam::Pose3& g);
  gtsam::Vector logmap(const gtsam::Pose3& g, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi_b) const;
  gtsam::Vector Adjoint(gtsam::Vector xi_b, Eigen::Ref<Eigen::MatrixXd> H_this,
                 Eigen::Ref<Eigen::MatrixXd> H_xib) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x, Eigen::Ref<Eigen::MatrixXd> H_this,
                          Eigen::Ref<Eigen::MatrixXd> H_x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Matrix adjointMap_(gtsam::Vector xi);
  static gtsam::Vector adjoint_(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Group Action on Point3
  gtsam::Point3 transformFrom(const gtsam::Point3& point) const;
  gtsam::Point3 transformFrom(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
                              Eigen::Ref<Eigen::MatrixXd> Hpoint) const;
  gtsam::Point3 transformTo(const gtsam::Point3& point) const;
  gtsam::Point3 transformTo(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
                            Eigen::Ref<Eigen::MatrixXd> Hpoint) const;

  // gtsam::Matrix versions
  gtsam::Matrix transformFrom(const gtsam::Matrix& points) const;
  gtsam::Matrix transformTo(const gtsam::Matrix& points) const;

  // Standard Interface
  gtsam::Rot3 rotation() const;
  gtsam::Rot3 rotation(Eigen::Ref<Eigen::MatrixXd> Hself) const;
  gtsam::Point3 translation() const;
  gtsam::Point3 translation(Eigen::Ref<Eigen::MatrixXd> Hself) const;
  double x() const;
  double y() const;
  double z() const;
  gtsam::Pose3 transformPoseFrom(const gtsam::Pose3& aTb) const;
  gtsam::Pose3 transformPoseFrom(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hself,
                                 Eigen::Ref<Eigen::MatrixXd> HaTb) const;
  gtsam::Pose3 transformPoseTo(const gtsam::Pose3& wTb) const;
  gtsam::Pose3 transformPoseTo(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hself,
                               Eigen::Ref<Eigen::MatrixXd> HwTb) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
               Eigen::Ref<Eigen::MatrixXd> Hpoint);
  double range(const gtsam::Pose3& pose);
  double range(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hself,
               Eigen::Ref<Eigen::MatrixXd> Hpose);
  gtsam::Unit3 bearing(const gtsam::Point3& point);
  gtsam::Unit3 bearing(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
                       Eigen::Ref<Eigen::MatrixXd> Hpoint);
  gtsam::Unit3 bearing(const gtsam::Pose3& pose);
  gtsam::Unit3 bearing(const gtsam::Pose3& pose, Eigen::Ref<Eigen::MatrixXd> Hself,
                       Eigen::Ref<Eigen::MatrixXd> Hpose);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/ExtendedPose3.h>
// An alias Se23 for ExtendedPose3 with k=2 is defined in python/gtsam/__init__.py
template <K = {2, 3, 4, 6}>
class ExtendedPose3 {
  // Standard Constructors
  ExtendedPose3();
  ExtendedPose3(const This& other);
  ExtendedPose3(const gtsam::Rot3& R, const gtsam::Matrix& x);
  ExtendedPose3(const gtsam::Matrix& T);

  // Testable
  void print(string s = "") const;
  bool equals(const This& other, double tol = 1e-9) const;

  // Access
  size_t k() const;
  gtsam::Rot3 rotation() const;
  gtsam::Point3 x(size_t i) const;
  gtsam::Matrix xMatrix() const;

  // Group
  static This Identity();
  This inverse() const;
  This compose(const This& g) const;
  This between(const This& g) const;

  // Operator Overloads
  This operator*(const This& other) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  This retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const This& g) const;

  // Lie Group
  static This Expmap(gtsam::Vector xi);
  static gtsam::Vector Logmap(const This& pose);
  static gtsam::Matrix ExpmapDerivative(gtsam::Vector xi);
  static gtsam::Matrix LogmapDerivative(gtsam::Vector xi);
  static gtsam::Matrix LogmapDerivative(const This& pose);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi_b) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // enabling serialization functionality
  void serialize() const;
};

class ExtendedPose3d {
  // Standard Constructors
  ExtendedPose3d();
  ExtendedPose3d(size_t k);
  ExtendedPose3d(const gtsam::ExtendedPose3d& other);
  ExtendedPose3d(const gtsam::Rot3& R, const gtsam::Matrix& x);
  ExtendedPose3d(const gtsam::Matrix& T);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::ExtendedPose3d& other, double tol = 1e-9) const;

  // Access
  static size_t Dimension(size_t k);
  size_t k() const;
  size_t dim() const;
  gtsam::Rot3 rotation() const;
  gtsam::Point3 x(size_t i) const;
  gtsam::Matrix xMatrix() const;

  // Group
  static gtsam::ExtendedPose3d Identity(size_t k = 0);
  gtsam::ExtendedPose3d inverse() const;
  gtsam::ExtendedPose3d compose(const gtsam::ExtendedPose3d& g) const;
  gtsam::ExtendedPose3d between(const gtsam::ExtendedPose3d& g) const;

  // Operator Overloads
  gtsam::ExtendedPose3d operator*(const gtsam::ExtendedPose3d& other) const;

  // Manifold
  gtsam::ExtendedPose3d retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::ExtendedPose3d& g) const;

  // Lie Group
  static gtsam::ExtendedPose3d Expmap(gtsam::Vector xi);
  static gtsam::Vector Logmap(const gtsam::ExtendedPose3d& pose);
  static gtsam::Matrix ExpmapDerivative(gtsam::Vector xi);
  static gtsam::Matrix LogmapDerivative(gtsam::Vector xi);
  static gtsam::Matrix LogmapDerivative(const gtsam::ExtendedPose3d& pose);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi_b) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/SL4.h>
class SL4 {
  // Standard constructors
  SL4();
  SL4(const gtsam::Matrix4& T);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SL4& sl4, double tol) const;

  // Group
  static gtsam::SL4 Identity();
  gtsam::SL4 inverse() const;
  gtsam::SL4 inverse(Eigen::Ref<Eigen::MatrixXd> H) const;
  gtsam::SL4 compose(const gtsam::SL4& sl4) const;
  gtsam::SL4 compose(const gtsam::SL4& sl4,
                     Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::SL4 between(const gtsam::SL4& sl4) const;
  gtsam::SL4 between(const gtsam::SL4& sl4,
                     Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Operator overload
  gtsam::SL4 operator*(const gtsam::SL4& sl4) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::SL4 retract(gtsam::Vector v,
                     Eigen::Ref<Eigen::MatrixXd> Horigin,
                     Eigen::Ref<Eigen::MatrixXd> Hv) const;
  gtsam::Vector localCoordinates(const gtsam::SL4& g,
                                 Eigen::Ref<Eigen::MatrixXd> Horigin,
                                 Eigen::Ref<Eigen::MatrixXd> Hp2) const;

  // Lie group
  static gtsam::SL4 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::SL4& g);
  gtsam::SL4 expmap(gtsam::Vector v);
  gtsam::Vector logmap(const gtsam::SL4& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Serialization
  void serialize() const;
};

// Used in Matlab wrapper
class Pose3Pairs {
  Pose3Pairs();
  size_t size() const;
  bool empty() const;
  gtsam::Pose3Pair at(size_t n) const;
  void push_back(const gtsam::Pose3Pair& pose_pair);
};

// Used in Matlab wrapper
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
  Unit3(double x, double y, double z);
  Unit3(const gtsam::Point2& p, double f);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Unit3& s, double tol) const;

  // Other functionality
  gtsam::Matrix basis() const;
  gtsam::Matrix basis(Eigen::Ref<Eigen::MatrixXd> H) const;
  gtsam::Matrix skew() const;
  gtsam::Point3 point3() const;
  gtsam::Point3 point3(Eigen::Ref<Eigen::MatrixXd> H) const;

  gtsam::Vector3 unitVector() const;
  gtsam::Vector3 unitVector(Eigen::Ref<Eigen::MatrixXd> H) const;
  double dot(const gtsam::Unit3& q) const;
  double dot(const gtsam::Unit3& q, Eigen::Ref<Eigen::MatrixXd> H1,
             Eigen::Ref<Eigen::MatrixXd> H2) const;
  gtsam::Vector2 errorVector(const gtsam::Unit3& q) const;
  gtsam::Vector2 errorVector(const gtsam::Unit3& q, Eigen::Ref<Eigen::MatrixXd> H_p,
                      Eigen::Ref<Eigen::MatrixXd> H_q) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Unit3 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::Unit3& s) const;
  gtsam::Unit3 FromPoint3(const gtsam::Point3& point) const;
  gtsam::Unit3 FromPoint3(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> H) const;

  // enabling serialization functionality
  void serialize() const;

  // enabling function to compare objects
  bool equals(const gtsam::Unit3& expected, double tol) const;
};

#include <gtsam/geometry/OrientedPlane3.h>
class OrientedPlane3 {
  // Standard constructors
  OrientedPlane3();
  OrientedPlane3(const gtsam::Unit3& n, double d);
  OrientedPlane3(const gtsam::Vector& vec);
  OrientedPlane3(double a, double b, double c, double d);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::OrientedPlane3& s, double tol = 1e-9) const;

  gtsam::OrientedPlane3 transform(const gtsam::Pose3& xr) const;
  gtsam::OrientedPlane3 transform(const gtsam::Pose3& xr,
                           Eigen::Ref<Eigen::MatrixXd> Hp,
                           Eigen::Ref<Eigen::MatrixXd> Hr) const;

  gtsam::Vector3 errorVector(const gtsam::OrientedPlane3& other) const;
  gtsam::Vector3 errorVector(const gtsam::OrientedPlane3& other,
                      Eigen::Ref<Eigen::MatrixXd> H1,
                      Eigen::Ref<Eigen::MatrixXd> H2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::OrientedPlane3 retract(const gtsam::Vector3& v) const;
  gtsam::OrientedPlane3 retract(const gtsam::Vector3& v,
                        Eigen::Ref<Eigen::MatrixXd> H) const;
  gtsam::Vector3 localCoordinates(const gtsam::OrientedPlane3& s) const;

  gtsam::Vector planeCoefficients() const;

  gtsam::Unit3 normal() const;
  gtsam::Unit3 normal(Eigen::Ref<Eigen::MatrixXd> H) const;
  double distance() const;
  double distance(Eigen::Ref<Eigen::MatrixXd> H) const;
};

#include <gtsam/geometry/EssentialMatrix.h>
class EssentialMatrix {
  // Standard Constructors
  EssentialMatrix(const gtsam::Rot3& aRb, const gtsam::Unit3& aTb);

  // Constructors from Pose3
  static gtsam::EssentialMatrix FromPose3(const gtsam::Pose3& _1P2_);

  static gtsam::EssentialMatrix FromPose3(const gtsam::Pose3& _1P2_,
                                          Eigen::Ref<Eigen::MatrixXd> H);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::EssentialMatrix& other, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::EssentialMatrix retract(gtsam::Vector xi) const;
  gtsam::Vector localCoordinates(const gtsam::EssentialMatrix& other) const;

  // Other methods:
  gtsam::Rot3 rotation() const;
  gtsam::Unit3 direction() const;
  gtsam::Matrix matrix() const;
  double error(gtsam::Vector vA, gtsam::Vector vB);
};

#include <gtsam/geometry/Similarity2.h>
class Similarity2 {
  // Standard Constructors
  Similarity2();
  Similarity2(double s);
  Similarity2(const gtsam::Rot2& R, const gtsam::Point2& t, double s);
  Similarity2(const gtsam::Matrix& R, const gtsam::Vector& t, double s);
  Similarity2(const gtsam::Matrix& T);

  gtsam::Point2 transformFrom(const gtsam::Point2& p) const;
  gtsam::Pose2 transformFrom(const gtsam::Pose2& T);

  static gtsam::Similarity2 Align(const gtsam::Point2Pairs& abPointPairs);
  static gtsam::Similarity2 Align(const gtsam::Pose2Pairs& abPosePairs);

  // Group
  static gtsam::Similarity2 Identity();
  gtsam::Similarity2 inverse() const;
  gtsam::Similarity2 compose(const gtsam::Similarity2& other) const;
  gtsam::Similarity2 between(const gtsam::Similarity2& other) const;

  // Operator Overloads
  gtsam::Similarity2 operator*(const gtsam::Similarity2& other) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Similarity2 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::Similarity2& t2) const;

  // Lie group
  static gtsam::Similarity2 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::Similarity2& S);
  gtsam::Similarity2 expmap(const gtsam::Vector& v);
  gtsam::Vector logmap(const gtsam::Similarity2& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Standard Interface
  bool equals(const gtsam::Similarity2& sim, double tol) const;
  void print(string s = "") const;
  gtsam::Matrix matrix() const;
  gtsam::Rot2& rotation();
  gtsam::Point2& translation();
  double scale() const;
};

#include <gtsam/geometry/Similarity3.h>
class Similarity3 {
  // Standard Constructors
  Similarity3();
  Similarity3(double s);
  Similarity3(const gtsam::Rot3& R, const gtsam::Point3& t, double s);
  Similarity3(const gtsam::Matrix& R, const gtsam::Vector& t, double s);
  Similarity3(const gtsam::Matrix& T);

  gtsam::Point3 transformFrom(const gtsam::Point3& p) const;
  gtsam::Pose3 transformFrom(const gtsam::Pose3& T);

  static gtsam::Similarity3 Align(const gtsam::Point3Pairs& abPointPairs);
  static gtsam::Similarity3 Align(const gtsam::Pose3Pairs& abPosePairs);

  // Group
  static gtsam::Similarity3 Identity();
  gtsam::Similarity3 inverse() const;
  gtsam::Similarity3 compose(const gtsam::Similarity3& other) const;
  gtsam::Similarity3 between(const gtsam::Similarity3& other) const;

  // Operator Overloads
  gtsam::Similarity3 operator*(const gtsam::Similarity3& other) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Similarity3 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::Similarity3& t2) const;

  // Lie group
  static gtsam::Similarity3 Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::Similarity3& s);
  gtsam::Similarity3 expmap(const gtsam::Vector& v);
  gtsam::Vector logmap(const gtsam::Similarity3& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // Standard Interface
  bool equals(const gtsam::Similarity3& sim, double tol) const;
  void print(string s = "") const;
  gtsam::Matrix matrix() const;
  gtsam::Rot3& rotation();
  gtsam::Point3& translation();
  double scale() const;
};

#include <gtsam/geometry/Event.h>
class Event {
  Event();
  Event(double t, const gtsam::Point3& p);
  Event(double t, double x, double y, double z);
  double time() const;
  gtsam::Point3 location() const;
  double height() const;
  void print(string s = "") const;
};


#include <gtsam/geometry/Gal3.h>
class Gal3 {
  // Standard Constructors
  Gal3();
  Gal3(const gtsam::Rot3& R, const gtsam::Point3& r, const gtsam::Vector3& v, double t);
  Gal3(const gtsam::Matrix5& M);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Gal3& other, double tol) const;

  // Manifold
  const gtsam::Rot3& attitude() const;
  const gtsam::Point3& position() const;
  const gtsam::Vector3& velocity() const;
  size_t dim() const;
  static size_t Dim();
  const double& time() const;
  gtsam::Gal3 retract(const gtsam::Vector10& xi) const;
  gtsam::Vector10 localCoordinates(const gtsam::Gal3& g) const;

  // Group
  const gtsam::Rot3& rotation() const;
  const gtsam::Point3& translation() const;
  static gtsam::Gal3 Identity();
  gtsam::Gal3 inverse() const;
  gtsam::Gal3 compose(const gtsam::Gal3& other) const;
  gtsam::Gal3 between(const gtsam::Gal3& other) const;
  gtsam::Event act(const gtsam::Event& e) const;
  double range(const gtsam::Point3& point) const;
  double range(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
               Eigen::Ref<Eigen::MatrixXd> Hpoint) const;
  gtsam::Unit3 bearing(const gtsam::Point3& point) const;
  gtsam::Unit3 bearing(const gtsam::Point3& point, Eigen::Ref<Eigen::MatrixXd> Hself,
                       Eigen::Ref<Eigen::MatrixXd> Hpoint) const;

  // Operator Overloads
  gtsam::Gal3 operator*(const gtsam::Gal3& other) const;

  // Lie Group
  static gtsam::Gal3 Expmap(const gtsam::Vector10& xi);
  static gtsam::Vector10 Logmap(const gtsam::Gal3& g);
  gtsam::Gal3 expmap(const gtsam::Vector10& xi);
  gtsam::Vector10 logmap(const gtsam::Gal3& g);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi) const;
  gtsam::Vector AdjointTranspose(gtsam::Vector x) const;
  static gtsam::Matrix adjointMap(gtsam::Vector xi);
  static gtsam::Vector adjoint(gtsam::Vector xi, gtsam::Vector y);
  static gtsam::Vector adjointTranspose(gtsam::Vector xi, gtsam::Vector y);
  gtsam::Vector vec() const;
  gtsam::Matrix matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);
};

#include <gtsam/geometry/BearingRange.h>
template <POSE, POINT, BEARING, RANGE>
class BearingRange {
  BearingRange(const BEARING& b, const RANGE& r);
  BEARING bearing() const;
  RANGE range() const;
  static This Measure(const POSE& a1, const POINT& a2);
  static BEARING MeasureBearing(const POSE& a1, const POINT& a2);
  static RANGE MeasureRange(const POSE& a1, const POINT& a2);
  void print(string str = "") const;
};

typedef gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>
    BearingRange2D;
typedef gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>
    BearingRangePose2;
typedef gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>
    BearingRange3D;
typedef gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>
    BearingRangePose3;

}  // namespace gtsam
