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
  gtsam::Vector3 vector() const;
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
  gtsam::Point3 normalize(
      const gtsam::Point3 &p,
      gtsam::OptionalJacobian<3, 3> H) const;
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
  gtsam::Rot2 inverse() const;
  gtsam::Rot2 compose(const gtsam::Rot2& p2) const;
  gtsam::Rot2 between(const gtsam::Rot2& p2) const;

  // Operator Overloads
  gtsam::Rot2 operator*(const gtsam::Rot2& p2) const;

  // Manifold
  static int Dim();
  int dim() const;
  gtsam::Rot2 retract(const gtsam::Vector1& v) const;
  gtsam::Rot2 retract(const gtsam::Vector1& v,
                      gtsam::OptionalJacobian<1, 1> H1,
                      gtsam::OptionalJacobian<1, 1> H2 = nullptr) const;
  gtsam::Vector1 localCoordinates(const gtsam::Rot2& p) const;
  gtsam::Vector1 localCoordinates(
      const gtsam::Rot2& p, gtsam::OptionalJacobian<1, 1> H1,
      gtsam::OptionalJacobian<1, 1> H2 = nullptr) const;

  // Lie Group
  static gtsam::Rot2 Expmap(
      const gtsam::Vector1& v,
      gtsam::OptionalJacobian<1, 1> H = nullptr);
  static gtsam::Vector1 Logmap(
      const gtsam::Rot2& r,
      gtsam::OptionalJacobian<1, 1> H = nullptr);
  gtsam::Rot2 expmap(const gtsam::Vector1& v) const;
  gtsam::Vector1 logmap(const gtsam::Rot2& g) const;

  // Matrix Lie Group
  gtsam::Matrix1 AdjointMap() const;
  gtsam::Vector1 Adjoint(
      const gtsam::Vector1& xi,
      gtsam::OptionalJacobian<1, 1> H_this = nullptr,
      gtsam::OptionalJacobian<1, 1> H_xi = nullptr) const;
  gtsam::Vector1 AdjointTranspose(
      const gtsam::Vector1& x,
      gtsam::OptionalJacobian<1, 1> H_this = nullptr,
      gtsam::OptionalJacobian<1, 1> H_x = nullptr) const;
  static gtsam::Matrix1 adjointMap(const gtsam::Vector1& xi);
  static gtsam::Vector1 adjoint(
      const gtsam::Vector1& xi, const gtsam::Vector1& y,
      gtsam::OptionalJacobian<1, 1> Hxi = nullptr,
      gtsam::OptionalJacobian<1, 1> H_y = nullptr);
  static gtsam::Vector1 adjointTranspose(
      const gtsam::Vector1& xi, const gtsam::Vector1& y,
      gtsam::OptionalJacobian<1, 1> Hxi = nullptr,
      gtsam::OptionalJacobian<1, 1> H_y = nullptr);
  gtsam::Vector4 vec(
      gtsam::OptionalJacobian<4, 1> H = nullptr) const;
  gtsam::Matrix2 matrix() const;
  static gtsam::Matrix2 Hat(const gtsam::Vector1& xi);
  static gtsam::Vector1 Vee(const gtsam::Matrix2& X);

  // Group Action on Point2
  gtsam::Point2 rotate(
      const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 1> H1 = nullptr,
      gtsam::OptionalJacobian<2, 2> H2 = nullptr) const;
  gtsam::Point2 unrotate(
      const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 1> H1 = nullptr,
      gtsam::OptionalJacobian<2, 2> H2 = nullptr) const;

  // Standard Interface
  static gtsam::Rot2 relativeBearing(
      const gtsam::Point2& d,
      gtsam::OptionalJacobian<1, 2> H = nullptr);
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
    gtsam::Vector3 applyLeft(
        const gtsam::Vector3& v,
        gtsam::OptionalJacobian<3, 3> Hw = nullptr,
        gtsam::OptionalJacobian<3, 3> Hv = nullptr) const;
    gtsam::Vector3 applyRight(
        const gtsam::Vector3& v,
        gtsam::OptionalJacobian<3, 3> Hw = nullptr,
        gtsam::OptionalJacobian<3, 3> Hv = nullptr) const;
    gtsam::Matrix3 frechet(const gtsam::Matrix3& X) const;
    gtsam::Matrix3 applyFrechet(const gtsam::Vector3& v) const;
  };

  class InvJKernel {
    gtsam::so3::Kernel J;  // holds the forward kernel
    gtsam::Matrix3 left() const;
    gtsam::Matrix3 right() const;
    gtsam::Vector3 applyLeft(
        const gtsam::Vector3& v,
        gtsam::OptionalJacobian<3, 3> Hw = nullptr,
        gtsam::OptionalJacobian<3, 3> Hv = nullptr) const;
    gtsam::Vector3 applyRight(
        const gtsam::Vector3& v,
        gtsam::OptionalJacobian<3, 3> Hw = nullptr,
        gtsam::OptionalJacobian<3, 3> Hv = nullptr) const;
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
    @pybind_lambda
    gtsam::so3::Kernel Rodrigues() const;
    @pybind_lambda
    gtsam::so3::Kernel Jacobian() const;
    @pybind_lambda
    gtsam::so3::InvJKernel InvJacobian() const;
    @pybind_lambda
    gtsam::so3::Kernel Gamma() const;

    // access to (lazily evaluated) coefficients
    double C() const;  // (1 - A) / theta^2
    double D() const;  // (1 - A/2B) / theta2
    double E() const;  // Coefficient for Gamma kernel

    // Use kernel if you need to apply
    gtsam::Matrix3 rightJacobian() const;
    gtsam::Matrix3 leftJacobian() const;
  };
}

class SO3 {
  // Standard Constructors
  SO3();
  SO3(gtsam::Matrix3 R);
  @pybind_lambda
  static gtsam::SO3 FromMatrix(gtsam::Matrix3 R);
  static gtsam::SO3 AxisAngle(const gtsam::Vector3& axis, double theta);
  static gtsam::SO3 ClosestTo(const gtsam::Matrix3& M);

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
  static int Dim();
  size_t dim() const;
  gtsam::SO3 retract(const gtsam::Vector3& v) const;
  gtsam::Vector3 localCoordinates(const gtsam::SO3& R) const;

  // Lie Group
  static gtsam::SO3 Expmap(
      const gtsam::Vector3& v,
      gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Vector3 Logmap(
      const gtsam::SO3& p,
      gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Matrix3 ExpmapDerivative(const gtsam::Vector3& omega);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Vector3& omega);
  gtsam::SO3 expmap(const gtsam::Vector3& v) const;
  gtsam::Vector3 logmap(const gtsam::SO3& g) const;

  // Matrix Lie Group
  gtsam::Matrix3 AdjointMap() const;
  gtsam::Vector3 Adjoint(
      const gtsam::Vector3& xi,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_xi = nullptr) const;
  gtsam::Vector3 AdjointTranspose(
      const gtsam::Vector3& x,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_x = nullptr) const;
  static gtsam::Matrix3 adjointMap(const gtsam::Vector3& xi);
  static gtsam::Vector3 adjoint(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  static gtsam::Vector3 adjointTranspose(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  gtsam::Vector9 vec(
      gtsam::OptionalJacobian<9, 3> H = nullptr) const;
  const gtsam::Matrix3& matrix() const;
  static gtsam::Matrix3 Hat(const gtsam::Vector3& xi);
  static gtsam::Vector3 Vee(const gtsam::Matrix3& X);
};

#include <gtsam/geometry/SO4.h>
class SO4 {
  // Standard Constructors
  SO4();
  SO4(gtsam::Matrix4 R);
  @pybind_lambda
  static gtsam::SO4 FromMatrix(gtsam::Matrix4 R);

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
  static int Dim();
  size_t dim() const;
  gtsam::SO4 retract(const gtsam::Vector6& v) const;
  gtsam::Vector6 localCoordinates(const gtsam::SO4& Q) const;

  // Lie Group
  static gtsam::SO4 Expmap(const gtsam::Vector6& v);
  static gtsam::Vector6 Logmap(const gtsam::SO4& p);
  gtsam::SO4 expmap(const gtsam::Vector6& v) const;
  gtsam::Vector6 logmap(const gtsam::SO4& g) const;

  // Matrix Lie Group
  gtsam::Matrix6 AdjointMap() const;
  gtsam::Vector6 Adjoint(
      const gtsam::Vector6& xi,
      gtsam::OptionalJacobian<6, 6> H_this = nullptr,
      gtsam::OptionalJacobian<6, 6> H_xi = nullptr) const;
  gtsam::Vector6 AdjointTranspose(
      const gtsam::Vector6& x,
      gtsam::OptionalJacobian<6, 6> H_this = nullptr,
      gtsam::OptionalJacobian<6, 6> H_x = nullptr) const;
  static gtsam::Matrix6 adjointMap(const gtsam::Vector6& xi);
  static gtsam::Vector6 adjoint(
      const gtsam::Vector6& xi, const gtsam::Vector6& y,
      gtsam::OptionalJacobian<6, 6> Hxi = nullptr,
      gtsam::OptionalJacobian<6, 6> H_y = nullptr);
  static gtsam::Vector6 adjointTranspose(
      const gtsam::Vector6& xi, const gtsam::Vector6& y,
      gtsam::OptionalJacobian<6, 6> Hxi = nullptr,
      gtsam::OptionalJacobian<6, 6> H_y = nullptr);
  gtsam::SO4::VectorN2 vec(
      gtsam::OptionalJacobian<16, 6> H = nullptr) const;
  const gtsam::Matrix4& matrix() const;
  static gtsam::Matrix4 Hat(const gtsam::Vector6& xi);
  static gtsam::Vector6 Vee(const gtsam::Matrix4& X);

};

#include <gtsam/geometry/SOn.h>
class SOn {
  // Standard Constructors
  SOn(size_t n);
  @pybind_lambda
  static gtsam::SOn FromMatrix(gtsam::Matrix R);
  @pybind_lambda
  static gtsam::SOn Lift(size_t n, gtsam::Matrix R);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::SOn& other, double tol) const;

  // Group
  @pybind_lambda
  static gtsam::SOn Identity();
  gtsam::SOn inverse() const;
  gtsam::SOn between(const gtsam::SOn& Q) const;
  gtsam::SOn compose(const gtsam::SOn& Q) const;

  // Operator Overloads
  gtsam::SOn operator*(const gtsam::SOn& Q) const;

  // Manifold
  static int Dim();
  size_t dim() const;
  gtsam::SOn retract(const gtsam::Vector& v) const;
  gtsam::Vector localCoordinates(const gtsam::SOn& Q) const;
  // Lie Group
  static gtsam::SOn Expmap(const gtsam::Vector& v);
  static gtsam::Vector Logmap(const gtsam::SOn& p);
  gtsam::SOn expmap(const gtsam::Vector& v) const;
  gtsam::Vector logmap(const gtsam::SOn& g) const;

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(
      const gtsam::Vector& xi,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_this = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi = nullptr) const;
  gtsam::Vector AdjointTranspose(
      const gtsam::Vector& x,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_this = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_x = nullptr) const;
  static gtsam::Matrix adjointMap(const gtsam::Vector& xi);
  static gtsam::Vector adjoint(
      const gtsam::Vector& xi, const gtsam::Vector& y,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hxi = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_y = nullptr);
  static gtsam::Vector adjointTranspose(
      const gtsam::Vector& xi, const gtsam::Vector& y,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hxi = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_y = nullptr);
  gtsam::Vector vec(
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = nullptr) const;
  const gtsam::Matrix& matrix() const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Quaternion.h>
class Quaternion {
  const double& w() const;
  const double& x() const;
  const double& y() const;
  const double& z() const;
  const gtsam::Quaternion::Coefficients& coeffs() const;
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
  static gtsam::Rot3 RzRyRx(
      double x, double y, double z,
      gtsam::OptionalJacobian<3, 1> Hx = nullptr,
      gtsam::OptionalJacobian<3, 1> Hy = nullptr,
      gtsam::OptionalJacobian<3, 1> Hz = nullptr);
  static gtsam::Rot3 RzRyRx(
      const gtsam::Vector& xyz,
      gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Rot3 Yaw(
      double t);  // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 Pitch(
      double t);  // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 Roll(
      double t);  // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 Ypr(
      double y, double p, double r,
      gtsam::OptionalJacobian<3, 1> Hy = nullptr,
      gtsam::OptionalJacobian<3, 1> Hp = nullptr,
      gtsam::OptionalJacobian<3, 1> Hr = nullptr);
  static gtsam::Rot3 Quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 AxisAngle(const gtsam::Point3& axis, double angle);
  static gtsam::Rot3 Rodrigues(const gtsam::Vector3& v);
  static gtsam::Rot3 Rodrigues(double wx, double wy, double wz);
  static gtsam::Rot3 ClosestTo(const gtsam::Matrix3& M);
  static bool IsValid(const gtsam::Matrix3& R, double tol = 1e-9);

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
  static int Dim();
  int dim() const;
  // gtsam::Rot3 retractCayley(gtsam::Vector v) const; // TODO, does not exist in both
  // gtsam::Matrix and Quaternion options
  gtsam::Rot3 retract(const gtsam::Vector3& v) const;
  gtsam::Vector3 localCoordinates(const gtsam::Rot3& p) const;

  // Lie group
  static gtsam::Rot3 Expmap(
      const gtsam::Vector3& v,
      gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Vector3 Logmap(
      const gtsam::Rot3& R,
      gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Matrix3 ExpmapDerivative(const gtsam::Vector3& omega);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Vector3& omega);
  gtsam::Rot3 expmap(const gtsam::Vector3& v) const;
  gtsam::Vector3 logmap(const gtsam::Rot3& g) const;

  // Matrix Lie Group
  gtsam::Matrix3 AdjointMap() const;
  gtsam::Vector3 Adjoint(
      const gtsam::Vector3& xi,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_xi = nullptr) const;
  gtsam::Vector3 AdjointTranspose(
      const gtsam::Vector3& x,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_x = nullptr) const;
  static gtsam::Matrix3 adjointMap(const gtsam::Vector3& xi);
  static gtsam::Vector3 adjoint(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  static gtsam::Vector3 adjointTranspose(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  gtsam::Vector9 vec(
      gtsam::OptionalJacobian<9, 3> H = nullptr) const;
  gtsam::Matrix3 matrix() const;
  static gtsam::Matrix3 Hat(const gtsam::Vector3& xi);
  static gtsam::Vector3 Vee(const gtsam::Matrix3& X);

  // Group Action on Point3
  gtsam::Point3 rotate(
      const gtsam::Point3& p,
      gtsam::OptionalJacobian<3, 3> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Point3 unrotate(
      const gtsam::Point3& p,
      gtsam::OptionalJacobian<3, 3> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;

  // Group action on Unit3
  gtsam::Unit3 rotate(const gtsam::Unit3& p,
                      gtsam::OptionalJacobian<2, 3> HR = nullptr,
                      gtsam::OptionalJacobian<2, 2> Hp = nullptr) const;
  gtsam::Unit3 unrotate(const gtsam::Unit3& p,
                        gtsam::OptionalJacobian<2, 3> HR = nullptr,
                        gtsam::OptionalJacobian<2, 2> Hp = nullptr) const;

  // Standard Interface
  gtsam::Matrix3 transpose() const;
  gtsam::Vector3 xyz(
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;
  gtsam::Vector3 ypr(
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;
  gtsam::Vector3 rpy(
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;
  double roll(gtsam::OptionalJacobian<1, 3> H = nullptr) const;
  double pitch(gtsam::OptionalJacobian<1, 3> H = nullptr) const;
  double yaw(gtsam::OptionalJacobian<1, 3> H = nullptr) const;
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
  static std::optional<gtsam::Pose2> Align(gtsam::ConstMatrixView a, gtsam::ConstMatrixView b);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose2& pose, double tol) const;

  // Group
  static gtsam::Pose2 Identity();
  gtsam::Pose2 inverse() const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2) const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2,
                       gtsam::OptionalJacobian<3, 3> H1,
                       gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2,
                       gtsam::OptionalJacobian<3, 3> H1,
                       gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;

  // Operator Overloads
  gtsam::Pose2 operator*(const gtsam::Pose2& p2) const;

  // Manifold
  static int Dim();
  int dim() const;
  gtsam::Pose2 retract(const gtsam::Vector3& v) const;
  gtsam::Pose2 retract(const gtsam::Vector3& v,
                      gtsam::OptionalJacobian<3, 3> H1,
                      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Vector3 localCoordinates(const gtsam::Pose2& p) const;
  gtsam::Vector3 localCoordinates(
      const gtsam::Pose2& p, gtsam::OptionalJacobian<3, 3> H1,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;

  // Lie Group
  static gtsam::Pose2 Expmap(const gtsam::Vector3& xi,
                             gtsam::OptionalJacobian<3, 3> H = nullptr);
  static gtsam::Vector3 Logmap(const gtsam::Pose2& p,
                               gtsam::OptionalJacobian<3, 3> H = nullptr);
  gtsam::Pose2 expmap(const gtsam::Vector3& v) const;
  gtsam::Pose2 expmap(const gtsam::Vector3& v,
                     gtsam::OptionalJacobian<3, 3> H1,
                     gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Vector3 logmap(const gtsam::Pose2& g) const;
  gtsam::Vector3 logmap(
      const gtsam::Pose2& g, gtsam::OptionalJacobian<3, 3> H1,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  static gtsam::Matrix3 ExpmapDerivative(const gtsam::Vector3& v);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Pose2& v);
  static gtsam::Matrix3 LogmapDerivative(const gtsam::Vector3& xi);

  // Matrix Lie Group
  gtsam::Matrix3 AdjointMap() const;
  gtsam::Vector3 Adjoint(
      const gtsam::Vector3& xi,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_xi = nullptr) const;
  gtsam::Vector3 AdjointTranspose(
      const gtsam::Vector3& x,
      gtsam::OptionalJacobian<3, 3> H_this = nullptr,
      gtsam::OptionalJacobian<3, 3> H_x = nullptr) const;
  static gtsam::Matrix3 adjointMap(const gtsam::Vector3& xi);
  static gtsam::Vector3 adjoint(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  static gtsam::Vector3 adjointTranspose(
      const gtsam::Vector3& xi, const gtsam::Vector3& y,
      gtsam::OptionalJacobian<3, 3> Hxi = nullptr,
      gtsam::OptionalJacobian<3, 3> H_y = nullptr);
  static gtsam::Matrix3 adjointMap_(const gtsam::Vector3& xi);
  static gtsam::Vector3 adjoint_(
      const gtsam::Vector3& xi, const gtsam::Vector3& y);
  gtsam::Vector9 vec(
      gtsam::OptionalJacobian<9, 3> H = nullptr) const;
  gtsam::Matrix3 matrix() const;
  static gtsam::Matrix3 Hat(const gtsam::Vector3& xi);
  static gtsam::Vector3 Vee(const gtsam::Matrix3& X);

  // Group Actions on Point2
  gtsam::Point2 transformFrom(const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 3> Dpose = nullptr,
      gtsam::OptionalJacobian<2, 2> Dpoint = nullptr) const;
  gtsam::Point2 transformTo(const gtsam::Point2& p,
      gtsam::OptionalJacobian<2, 3> Dpose = nullptr,
      gtsam::OptionalJacobian<2, 2> Dpoint = nullptr) const;

  // gtsam::Matrix versions
  gtsam::Matrix transformFrom(gtsam::ConstMatrixView points) const;
  gtsam::Matrix transformTo(gtsam::ConstMatrixView points) const;

  // Standard Interface
  double x() const;
  double y() const;
  double theta() const;
  gtsam::Rot2 bearing(const gtsam::Point2& point,
      gtsam::OptionalJacobian<1, 3> H1 = nullptr,
      gtsam::OptionalJacobian<1, 2> H2 = nullptr) const;
  double range(const gtsam::Point2& point,
      gtsam::OptionalJacobian<1, 3> H1 = nullptr,
      gtsam::OptionalJacobian<1, 2> H2 = nullptr) const;
  const gtsam::Point2& translation(
      gtsam::OptionalJacobian<2, 3> Hself = nullptr) const;
  const gtsam::Rot2& rotation(
      gtsam::OptionalJacobian<1, 3> Hself = nullptr) const;

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
  static std::optional<gtsam::Pose3> Align(gtsam::ConstMatrixView a, gtsam::ConstMatrixView b);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 Identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 inverse(gtsam::OptionalJacobian<6, 6> H) const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose) const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose,
                       gtsam::OptionalJacobian<6, 6> H1,
                       gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose,
                       gtsam::OptionalJacobian<6, 6> H1,
                       gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;
  gtsam::Pose3 slerp(double t, const gtsam::Pose3& other,
                     gtsam::OptionalJacobian<6, 6> Hx = nullptr,
                     gtsam::OptionalJacobian<6, 6> Hy = nullptr) const;

  // Operator Overloads
  gtsam::Pose3 operator*(const gtsam::Pose3& pose) const;

  // Manifold
  static int Dim();
  size_t dim() const;
  gtsam::Pose3 retract(const gtsam::Vector6& v) const;
  @pybind_lambda
  gtsam::Pose3 retract(const gtsam::Vector6& v,
                      gtsam::OptionalJacobian<6, 6> H1,
                      gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;
  gtsam::Vector6 localCoordinates(const gtsam::Pose3& pose) const;
  @pybind_lambda
  gtsam::Vector6 localCoordinates(
      const gtsam::Pose3& pose, gtsam::OptionalJacobian<6, 6> H1,
      gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;

  // Lie Group
  static gtsam::Pose3 Expmap(const gtsam::Vector6& xi,
                             gtsam::OptionalJacobian<6, 6> Hxi = nullptr);
  static gtsam::Vector6 Logmap(const gtsam::Pose3& p,
                               gtsam::OptionalJacobian<6, 6> Hpose = nullptr);
  static gtsam::Matrix6 ExpmapDerivative(const gtsam::Vector6& xi);
  static gtsam::Matrix6 LogmapDerivative(const gtsam::Vector6& xi);
  static gtsam::Matrix6 LogmapDerivative(const gtsam::Pose3& xi);
  gtsam::Pose3 expmap(const gtsam::Vector6& v) const;
  gtsam::Pose3 expmap(const gtsam::Vector6& v,
                     gtsam::OptionalJacobian<6, 6> H1,
                     gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;
  
  gtsam::Vector6 logmap(const gtsam::Pose3& g) const;
  gtsam::Vector6 logmap(
      const gtsam::Pose3& g, gtsam::OptionalJacobian<6, 6> H1,
      gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;

  // Matrix Lie Group
  gtsam::Matrix6 AdjointMap() const;
  gtsam::Vector6 Adjoint(
      const gtsam::Vector6& xi_b,
      gtsam::OptionalJacobian<6, 6> H_this = nullptr,
      gtsam::OptionalJacobian<6, 6> H_xi = nullptr) const;
  gtsam::Vector6 AdjointTranspose(
      const gtsam::Vector6& x,
      gtsam::OptionalJacobian<6, 6> H_this = nullptr,
      gtsam::OptionalJacobian<6, 6> H_x = nullptr) const;
  static gtsam::Matrix6 adjointMap(const gtsam::Vector6& xi);
  static gtsam::Vector6 adjoint(
      const gtsam::Vector6& xi, const gtsam::Vector6& y,
      gtsam::OptionalJacobian<6, 6> Hxi = nullptr,
      gtsam::OptionalJacobian<6, 6> H_y = nullptr);
  static gtsam::Vector6 adjointTranspose(
      const gtsam::Vector6& xi, const gtsam::Vector6& y,
      gtsam::OptionalJacobian<6, 6> Hxi = nullptr,
      gtsam::OptionalJacobian<6, 6> H_y = nullptr);
  static gtsam::Matrix6 adjointMap_(const gtsam::Vector6& xi);
  static gtsam::Vector6 adjoint_(
      const gtsam::Vector6& xi, const gtsam::Vector6& y);
  gtsam::Pose3::Vector16 vec(
      gtsam::OptionalJacobian<16, 6> H = nullptr) const;
  gtsam::Matrix4 matrix() const;
  static gtsam::Matrix4 Hat(const gtsam::Vector6& xi);
  static gtsam::Vector6 Vee(const gtsam::Matrix4& X);

  // Group Action on Point3
  gtsam::Point3 transformFrom(
      const gtsam::Point3& point,
      gtsam::OptionalJacobian<3, 6> Hself = nullptr,
      gtsam::OptionalJacobian<3, 3> Hpoint = nullptr) const;
  gtsam::Point3 transformTo(
      const gtsam::Point3& point,
      gtsam::OptionalJacobian<3, 6> Hself = nullptr,
      gtsam::OptionalJacobian<3, 3> Hpoint = nullptr) const;

  // gtsam::Matrix versions
  gtsam::Matrix transformFrom(gtsam::ConstMatrixView points) const;
  gtsam::Matrix transformTo(gtsam::ConstMatrixView points) const;

  // Standard Interface
  const gtsam::Rot3& rotation(
      gtsam::OptionalJacobian<3, 6> Hself = nullptr) const;
  const gtsam::Point3& translation(
      gtsam::OptionalJacobian<3, 6> Hself = nullptr) const;
  double x() const;
  double y() const;
  double z() const;
  gtsam::Pose3 transformPoseFrom(
      const gtsam::Pose3& aTb,
      gtsam::OptionalJacobian<6, 6> Hself = nullptr,
      gtsam::OptionalJacobian<6, 6> HaTb = nullptr) const;
  gtsam::Pose3 transformPoseTo(
      const gtsam::Pose3& wTb,
      gtsam::OptionalJacobian<6, 6> Hself = nullptr,
      gtsam::OptionalJacobian<6, 6> HwTb = nullptr) const;
  double range(const gtsam::Point3& point,
               gtsam::OptionalJacobian<1, 6> Hself = nullptr,
               gtsam::OptionalJacobian<1, 3> Hpoint = nullptr) const;
  double range(const gtsam::Pose3& pose,
               gtsam::OptionalJacobian<1, 6> Hself = nullptr,
               gtsam::OptionalJacobian<1, 6> Hpose = nullptr) const;
  gtsam::Unit3 bearing(const gtsam::Point3& point,
                       gtsam::OptionalJacobian<2, 6> Hself = nullptr,
                       gtsam::OptionalJacobian<2, 3> Hpoint = nullptr) const;
  gtsam::Unit3 bearing(const gtsam::Pose3& pose,
                       gtsam::OptionalJacobian<2, 6> Hself = nullptr,
                       gtsam::OptionalJacobian<2, 6> Hpose = nullptr) const;

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
  ExtendedPose3(const gtsam::Rot3& R, const gtsam::This::Matrix3K& x);
  ExtendedPose3(const gtsam::This::MatrixRep& T);

  // Testable
  void print(string s = "") const;
  bool equals(const This& other, double tol = 1e-9) const;

  // Access
  size_t k() const;
  const gtsam::Rot3& rotation(
      gtsam::This::ComponentJacobian H = nullptr) const;
  gtsam::Point3 x(
      size_t i, gtsam::This::ComponentJacobian H = nullptr) const;
  const gtsam::This::Matrix3K& xMatrix() const;

  // Group
  static This Identity();
  This inverse() const;
  This compose(const This& g) const;
  This between(const This& g) const;

  // Operator Overloads
  This operator*(const This& other) const;

  // Manifold
  static int Dim();
  size_t dim() const;
  This retract(const gtsam::This::TangentVector& v) const;
  gtsam::This::TangentVector localCoordinates(const This& g) const;

  // Lie Group
  static This Expmap(
      const gtsam::This::TangentVector& xi,
      gtsam::This::ChartJacobian Hxi = nullptr);
  static gtsam::This::TangentVector Logmap(
      const This& pose, gtsam::This::ChartJacobian Hpose = nullptr);
  static gtsam::This::Jacobian ExpmapDerivative(
      const gtsam::This::TangentVector& xi);
  static gtsam::This::Jacobian LogmapDerivative(
      const gtsam::This::TangentVector& xi);
  static gtsam::This::Jacobian LogmapDerivative(const This& pose);

  // Matrix Lie Group
  gtsam::This::Jacobian AdjointMap() const;
  gtsam::This::TangentVector Adjoint(
      const gtsam::This::TangentVector& xi_b,
      gtsam::This::ChartJacobian H_this = nullptr,
      gtsam::This::ChartJacobian H_xi = nullptr) const;
  gtsam::This::TangentVector AdjointTranspose(
      const gtsam::This::TangentVector& x,
      gtsam::This::ChartJacobian H_this = nullptr,
      gtsam::This::ChartJacobian H_x = nullptr) const;
  static gtsam::This::Jacobian adjointMap(
      const gtsam::This::TangentVector& xi);
  static gtsam::This::TangentVector adjoint(
      const gtsam::This::TangentVector& xi,
      const gtsam::This::TangentVector& y,
      gtsam::This::ChartJacobian Hxi = nullptr,
      gtsam::This::ChartJacobian H_y = nullptr);
  static gtsam::This::TangentVector adjointTranspose(
      const gtsam::This::TangentVector& xi,
      const gtsam::This::TangentVector& y,
      gtsam::This::ChartJacobian Hxi = nullptr,
      gtsam::This::ChartJacobian H_y = nullptr);
  gtsam::This::Vectorized vec(
      gtsam::This::VectorizedJacobian H = nullptr) const;
  gtsam::This::MatrixRep matrix() const;
  static gtsam::This::LieAlgebra Hat(
      const gtsam::This::TangentVector& xi);
  static gtsam::This::TangentVector Vee(
      const gtsam::This::LieAlgebra& X);

  // enabling serialization functionality
  void serialize() const;
};

class ExtendedPose3d {
  // Standard Constructors
  ExtendedPose3d();
  ExtendedPose3d(size_t k);
  ExtendedPose3d(const gtsam::ExtendedPose3d& other);
  ExtendedPose3d(
      const gtsam::Rot3& R,
      const gtsam::ExtendedPose3d::Matrix3K& x);
  ExtendedPose3d(const gtsam::Matrix& T);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::ExtendedPose3d& other, double tol = 1e-9) const;

  // Access
  static size_t Dimension(size_t k);
  size_t k() const;
  size_t dim() const;
  const gtsam::Rot3& rotation(
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = nullptr) const;
  gtsam::Point3 x(
      size_t i,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = nullptr) const;
  const gtsam::ExtendedPose3d::Matrix3K& xMatrix() const;

  // Group
  static gtsam::ExtendedPose3d Identity(size_t k = 0);
  gtsam::ExtendedPose3d inverse() const;
  gtsam::ExtendedPose3d compose(const gtsam::ExtendedPose3d& g) const;
  gtsam::ExtendedPose3d between(const gtsam::ExtendedPose3d& g) const;

  // Operator Overloads
  gtsam::ExtendedPose3d operator*(const gtsam::ExtendedPose3d& other) const;

  // Manifold
  gtsam::ExtendedPose3d retract(
      const gtsam::Vector& v,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = nullptr) const;
  gtsam::Vector localCoordinates(const gtsam::ExtendedPose3d& g) const;

  // Lie Group
  static gtsam::ExtendedPose3d Expmap(
      const gtsam::Vector& xi,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hxi = nullptr);
  static gtsam::Vector Logmap(
      const gtsam::ExtendedPose3d& pose,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hpose = nullptr);
  static gtsam::Matrix ExpmapDerivative(const gtsam::Vector& xi);
  static gtsam::Matrix LogmapDerivative(const gtsam::Vector& xi);
  static gtsam::Matrix LogmapDerivative(const gtsam::ExtendedPose3d& pose);

  // Matrix Lie Group
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(
      const gtsam::Vector& xi_b,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_this = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi = nullptr) const;
  gtsam::Vector AdjointTranspose(
      const gtsam::Vector& x,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_this = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_x = nullptr) const;
  static gtsam::Matrix adjointMap(const gtsam::Vector& xi);
  static gtsam::Vector adjoint(
      const gtsam::Vector& xi, const gtsam::Vector& y,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hxi = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_y = nullptr);
  static gtsam::Vector adjointTranspose(
      const gtsam::Vector& xi, const gtsam::Vector& y,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> Hxi = nullptr,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_y = nullptr);
  gtsam::Vector vec(
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = nullptr) const;
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
  gtsam::SL4 inverse(gtsam::OptionalJacobian<15, 15> H) const;
  gtsam::SL4 compose(const gtsam::SL4& sl4) const;
  gtsam::SL4 compose(const gtsam::SL4& sl4,
                     gtsam::OptionalJacobian<15, 15> H1,
                     gtsam::OptionalJacobian<15, 15> H2 = nullptr) const;
  gtsam::SL4 between(const gtsam::SL4& sl4) const;
  gtsam::SL4 between(const gtsam::SL4& sl4,
                     gtsam::OptionalJacobian<15, 15> H1,
                     gtsam::OptionalJacobian<15, 15> H2 = nullptr) const;

  // Operator overload
  gtsam::SL4 operator*(const gtsam::SL4& sl4) const;

  // Manifold
  static int Dim();
  int dim() const;
  gtsam::SL4 retract(const gtsam::Vector15& v) const;
  gtsam::Vector15 localCoordinates(const gtsam::SL4& g) const;

  // Lie group
  static gtsam::SL4 Expmap(const gtsam::Vector& xi);
  static gtsam::Vector Logmap(const gtsam::SL4& p);
  gtsam::SL4 expmap(const gtsam::Vector15& v) const;
  gtsam::Vector15 logmap(const gtsam::SL4& g) const;

  // Matrix Lie Group
  gtsam::SL4::Jacobian AdjointMap() const;
  gtsam::Vector15 Adjoint(
      const gtsam::Vector15& xi,
      gtsam::OptionalJacobian<15, 15> H_this = nullptr,
      gtsam::OptionalJacobian<15, 15> H_xi = nullptr) const;
  gtsam::Vector15 AdjointTranspose(
      const gtsam::Vector15& x,
      gtsam::OptionalJacobian<15, 15> H_this = nullptr,
      gtsam::OptionalJacobian<15, 15> H_x = nullptr) const;
  static gtsam::SL4::Jacobian adjointMap(
      const gtsam::Vector15& xi);
  static gtsam::Vector15 adjoint(
      const gtsam::Vector15& xi, const gtsam::Vector15& y,
      gtsam::OptionalJacobian<15, 15> Hxi = nullptr,
      gtsam::OptionalJacobian<15, 15> H_y = nullptr);
  static gtsam::Vector15 adjointTranspose(
      const gtsam::Vector15& xi, const gtsam::Vector15& y,
      gtsam::OptionalJacobian<15, 15> Hxi = nullptr,
      gtsam::OptionalJacobian<15, 15> H_y = nullptr);
  gtsam::SL4::Vectorized vec(
      gtsam::OptionalJacobian<16, 15> H = nullptr) const;
  const gtsam::Matrix44& matrix() const;
  static gtsam::Matrix4 Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix4& X);

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
  const gtsam::Matrix32& basis(
      gtsam::OptionalJacobian<6, 2> H = nullptr) const;
  gtsam::Matrix3 skew() const;
  gtsam::Point3 point3(gtsam::OptionalJacobian<3, 2> H = nullptr) const;

  gtsam::Vector3 unitVector(gtsam::OptionalJacobian<3, 2> H = nullptr) const;
  double dot(const gtsam::Unit3& q,
             gtsam::OptionalJacobian<1, 2> H1 = nullptr,
             gtsam::OptionalJacobian<1, 2> H2 = nullptr) const;
  gtsam::Vector2 errorVector(
      const gtsam::Unit3& q,
      gtsam::OptionalJacobian<2, 2> H_p = nullptr,
      gtsam::OptionalJacobian<2, 2> H_q = nullptr) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Unit3 retract(
      const gtsam::Vector2& v,
      gtsam::OptionalJacobian<2, 2> H = nullptr) const;
  gtsam::Vector2 localCoordinates(const gtsam::Unit3& s) const;
  static gtsam::Unit3 FromPoint3(
      const gtsam::Point3& point,
      gtsam::OptionalJacobian<2, 3> H = nullptr);

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

  gtsam::OrientedPlane3 transform(const gtsam::Pose3& xr,
      gtsam::OptionalJacobian<3, 3> Hp = nullptr,
      gtsam::OptionalJacobian<3, 6> Hr = nullptr) const;

  gtsam::Vector3 errorVector(const gtsam::OrientedPlane3& other,
      gtsam::OptionalJacobian<3, 3> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::OrientedPlane3 retract(const gtsam::Vector3& v,
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;
  gtsam::Vector3 localCoordinates(const gtsam::OrientedPlane3& s) const;

  gtsam::Vector4 planeCoefficients() const;

  gtsam::Unit3 normal(gtsam::OptionalJacobian<2, 3> H = nullptr) const;
  double distance(gtsam::OptionalJacobian<1, 3> H = nullptr) const;
};

#include <gtsam/geometry/EssentialMatrix.h>
class EssentialMatrix {
  // Standard Constructors
  EssentialMatrix(const gtsam::Rot3& aRb, const gtsam::Unit3& aTb);

  // Constructors from Pose3
  static gtsam::EssentialMatrix FromPose3(const gtsam::Pose3& _1P2_,
      gtsam::OptionalJacobian<5, 6> H = nullptr);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::EssentialMatrix& other, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::EssentialMatrix retract(const gtsam::Vector5& xi) const;
  gtsam::Vector5 localCoordinates(
      const gtsam::EssentialMatrix& other) const;

  // Other methods:
  const gtsam::Rot3& rotation() const;
  const gtsam::Unit3& direction() const;
  const gtsam::Matrix3& matrix() const;
  double error(
      const gtsam::Vector3& vA, const gtsam::Vector3& vB,
      gtsam::OptionalJacobian<1, 5> H = nullptr) const;
};

#include <gtsam/geometry/Similarity2.h>
class Similarity2 {
  // Standard Constructors
  Similarity2();
  Similarity2(double s);
  Similarity2(const gtsam::Rot2& R, const gtsam::Point2& t, double s);
  Similarity2(const gtsam::Matrix2& R, const gtsam::Vector2& t, double s);
  Similarity2(const gtsam::Matrix3& T);

  gtsam::Point2 transformFrom(const gtsam::Point2& p) const;
  gtsam::Pose2 transformFrom(const gtsam::Pose2& T) const;

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
  int dim() const;
  static int Dim();
  gtsam::Similarity2 retract(const gtsam::Vector4& v) const;
  gtsam::Vector4 localCoordinates(const gtsam::Similarity2& t2) const;

  // Lie group
  static gtsam::Similarity2 Expmap(const gtsam::Vector4& v);
  static gtsam::Vector4 Logmap(const gtsam::Similarity2& S);
  gtsam::Similarity2 expmap(const gtsam::Vector4& v) const;
  gtsam::Vector4 logmap(const gtsam::Similarity2& g) const;

  // Matrix Lie Group
  gtsam::Matrix4 AdjointMap() const;
  gtsam::Vector4 Adjoint(
      const gtsam::Vector4& xi,
      gtsam::OptionalJacobian<4, 4> H_this = nullptr,
      gtsam::OptionalJacobian<4, 4> H_xi = nullptr) const;
  gtsam::Vector4 AdjointTranspose(
      const gtsam::Vector4& x,
      gtsam::OptionalJacobian<4, 4> H_this = nullptr,
      gtsam::OptionalJacobian<4, 4> H_x = nullptr) const;
  static gtsam::Matrix4 adjointMap(const gtsam::Vector4& xi);
  static gtsam::Vector4 adjoint(
      const gtsam::Vector4& xi, const gtsam::Vector4& y,
      gtsam::OptionalJacobian<4, 4> Hxi = nullptr,
      gtsam::OptionalJacobian<4, 4> H_y = nullptr);
  static gtsam::Vector4 adjointTranspose(
      const gtsam::Vector4& xi, const gtsam::Vector4& y,
      gtsam::OptionalJacobian<4, 4> Hxi = nullptr,
      gtsam::OptionalJacobian<4, 4> H_y = nullptr);
  gtsam::Vector9 vec(
      gtsam::OptionalJacobian<9, 4> H = nullptr) const;
  gtsam::Matrix3 matrix() const;
  static gtsam::Matrix3 Hat(const gtsam::Vector4& xi);
  static gtsam::Vector4 Vee(const gtsam::Matrix3& X);

  // Standard Interface
  bool equals(const gtsam::Similarity2& sim, double tol) const;
  void print(string s = "") const;
  gtsam::Rot2 rotation() const;
  gtsam::Point2 translation() const;
  double scale() const;
};

#include <gtsam/geometry/Similarity3.h>
class Similarity3 {
  // Standard Constructors
  Similarity3();
  Similarity3(double s);
  Similarity3(const gtsam::Rot3& R, const gtsam::Point3& t, double s);
  Similarity3(const gtsam::Matrix3& R, const gtsam::Vector3& t, double s);
  Similarity3(const gtsam::Matrix4& T);

  gtsam::Point3 transformFrom(
      const gtsam::Point3& p,
      gtsam::OptionalJacobian<3, 7> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Pose3 transformFrom(
      const gtsam::Pose3& T,
      gtsam::OptionalJacobian<6, 7> H1 = nullptr,
      gtsam::OptionalJacobian<6, 6> H2 = nullptr) const;

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
  int dim() const;
  static int Dim();
  gtsam::Similarity3 retract(const gtsam::Vector7& v) const;
  gtsam::Vector7 localCoordinates(const gtsam::Similarity3& t2) const;

  // Lie group
  static gtsam::Similarity3 Expmap(const gtsam::Vector7& v);
  static gtsam::Vector7 Logmap(const gtsam::Similarity3& s);
  gtsam::Similarity3 expmap(const gtsam::Vector7& v) const;
  gtsam::Vector7 logmap(const gtsam::Similarity3& g) const;

  // Matrix Lie Group
  gtsam::Matrix7 AdjointMap() const;
  gtsam::Vector7 Adjoint(
      const gtsam::Vector7& xi,
      gtsam::OptionalJacobian<7, 7> H_this = nullptr,
      gtsam::OptionalJacobian<7, 7> H_xi = nullptr) const;
  gtsam::Vector7 AdjointTranspose(
      const gtsam::Vector7& x,
      gtsam::OptionalJacobian<7, 7> H_this = nullptr,
      gtsam::OptionalJacobian<7, 7> H_x = nullptr) const;
  static gtsam::Matrix7 adjointMap(const gtsam::Vector7& xi);
  static gtsam::Vector7 adjoint(
      const gtsam::Vector7& xi, const gtsam::Vector7& y,
      gtsam::OptionalJacobian<7, 7> Hxi = nullptr,
      gtsam::OptionalJacobian<7, 7> H_y = nullptr);
  static gtsam::Vector7 adjointTranspose(
      const gtsam::Vector7& xi, const gtsam::Vector7& y,
      gtsam::OptionalJacobian<7, 7> Hxi = nullptr,
      gtsam::OptionalJacobian<7, 7> H_y = nullptr);
  gtsam::Similarity3::Vector16 vec(
      gtsam::OptionalJacobian<16, 7> H = nullptr) const;
  gtsam::Matrix4 matrix() const;
  static gtsam::Matrix4 Hat(const gtsam::Vector7& xi);
  static gtsam::Vector7 Vee(const gtsam::Matrix4& X);

  // Standard Interface
  bool equals(const gtsam::Similarity3& sim, double tol) const;
  void print(string s = "") const;
  gtsam::Rot3 rotation(
      gtsam::OptionalJacobian<3, 7> Hself = nullptr) const;
  gtsam::Point3 translation(
      gtsam::OptionalJacobian<3, 7> Hself = nullptr) const;
  double scale(gtsam::OptionalJacobian<1, 7> Hself = nullptr) const;
};

#include <gtsam/geometry/Event.h>
class Event {
  Event();
  Event(double t, const gtsam::Point3& p);
  Event(double t, double x, double y, double z);
  double time() const;
  gtsam::Point3 location() const;
  double height(gtsam::OptionalJacobian<1, 4> H = nullptr) const;
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
  const gtsam::Rot3& attitude(
      gtsam::OptionalJacobian<3, 10> H = nullptr) const;
  const gtsam::Point3& position(
      gtsam::OptionalJacobian<3, 10> H = nullptr) const;
  const gtsam::Vector3& velocity(
      gtsam::OptionalJacobian<3, 10> H = nullptr) const;
  int dim() const;
  static int Dim();
  const double& time(
      gtsam::OptionalJacobian<1, 10> H = nullptr) const;
  gtsam::Gal3 retract(const gtsam::Vector10& xi) const;
  gtsam::Vector10 localCoordinates(const gtsam::Gal3& g) const;

  // Group
  const gtsam::Rot3& rotation(
      gtsam::OptionalJacobian<3, 10> H = nullptr) const;
  const gtsam::Point3& translation(
      gtsam::OptionalJacobian<3, 10> H = nullptr) const;
  static gtsam::Gal3 Identity();
  gtsam::Gal3 inverse() const;
  gtsam::Gal3 compose(const gtsam::Gal3& other) const;
  gtsam::Gal3 between(const gtsam::Gal3& other) const;
  gtsam::Event act(
      const gtsam::Event& e,
      gtsam::OptionalJacobian<4, 10> Hself = nullptr,
      gtsam::OptionalJacobian<4, 4> He = nullptr) const;
  double range(const gtsam::Point3& point,
               gtsam::OptionalJacobian<1, 10> Hself = nullptr,
               gtsam::OptionalJacobian<1, 3> Hpoint = nullptr) const;
  gtsam::Unit3 bearing(const gtsam::Point3& point,
                       gtsam::OptionalJacobian<2, 10> Hself = nullptr,
                       gtsam::OptionalJacobian<2, 3> Hpoint = nullptr) const;

  // Operator Overloads
  gtsam::Gal3 operator*(const gtsam::Gal3& other) const;

  // Lie Group
  static gtsam::Gal3 Expmap(
      const gtsam::Vector10& xi,
      gtsam::OptionalJacobian<10, 10> Hxi = nullptr);
  static gtsam::Vector10 Logmap(
      const gtsam::Gal3& g,
      gtsam::OptionalJacobian<10, 10> Hg = nullptr);
  gtsam::Gal3 expmap(const gtsam::Vector10& xi) const;
  gtsam::Vector10 logmap(const gtsam::Gal3& g) const;

  // Matrix Lie Group
  gtsam::Gal3::Jacobian AdjointMap() const;
  gtsam::Vector10 Adjoint(
      const gtsam::Vector10& xi,
      gtsam::OptionalJacobian<10, 10> H_this = nullptr,
      gtsam::OptionalJacobian<10, 10> H_xi = nullptr) const;
  gtsam::Vector10 AdjointTranspose(
      const gtsam::Vector10& x,
      gtsam::OptionalJacobian<10, 10> H_this = nullptr,
      gtsam::OptionalJacobian<10, 10> H_x = nullptr) const;
  static gtsam::Gal3::Jacobian adjointMap(
      const gtsam::Vector10& xi);
  static gtsam::Vector10 adjoint(
      const gtsam::Vector10& xi, const gtsam::Vector10& y,
      gtsam::OptionalJacobian<10, 10> Hxi = nullptr,
      gtsam::OptionalJacobian<10, 10> H_y = nullptr);
  static gtsam::Vector10 adjointTranspose(
      const gtsam::Vector10& xi, const gtsam::Vector10& y,
      gtsam::OptionalJacobian<10, 10> Hxi = nullptr,
      gtsam::OptionalJacobian<10, 10> H_y = nullptr);
  gtsam::Gal3::Vector25 vec(
      gtsam::OptionalJacobian<25, 10> H = nullptr) const;
  gtsam::Matrix5 matrix() const;
  static gtsam::Matrix5 Hat(const gtsam::Vector10& xi);
  static gtsam::Vector10 Vee(const gtsam::Matrix5& X);
};

#include <gtsam/geometry/BearingRange.h>
template <POSE, POINT, BEARING, RANGE>
class BearingRange {
  BearingRange(const BEARING& b, const RANGE& r);
  const BEARING& bearing() const;
  const RANGE& range() const;
  static This Measure(
      const POSE& a1, const POINT& a2,
      gtsam::This::OptionalJacobian1 H1 = nullptr,
      gtsam::This::OptionalJacobian2 H2 = nullptr);
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
