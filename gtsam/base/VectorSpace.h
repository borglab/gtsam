/*
 * VectorSpace.h
 *
 * @date December 21, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Lie.h>

namespace gtsam {

/// tag to assert a type is a vector space
struct vector_space_tag: public lie_group_tag {
};

template<typename T> struct traits;

namespace internal {

/// VectorSpaceTraits Implementation for Fixed sizes
template<class Class, int N>
struct VectorSpaceImpl {

  /// @name Manifold
  /// @{
  typedef Eigen::Matrix<double, N, 1> TangentVector;
  typedef OptionalJacobian<N, N> ChartJacobian;
  typedef Eigen::Matrix<double, N, N> Jacobian;
  static int GetDimension(const Class&) { return N;}

  static TangentVector Local(const Class& origin, const Class& other,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = - Jacobian::Identity();
    if (H2) *H2 = Jacobian::Identity();
    Class v = other-origin;
    return v.vector();
  }

  static Class Retract(const Class& origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Jacobian::Identity();
    if (H2) *H2 = Jacobian::Identity();
    return origin + v;
  }

  /// @}

  /// @name Lie Group
  /// @{

  static TangentVector Logmap(const Class& m, ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Jacobian::Identity();
    return m.vector();
  }

  static Class Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    if (Hv)  *Hv = Jacobian::Identity();
    return Class(v);
  }

  static Class Compose(const Class& v1, const Class& v2, ChartJacobian H1 = boost::none,
      ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Jacobian::Identity();
    if (H2) *H2 = Jacobian::Identity();
    return v1 + v2;
  }

  static Class Between(const Class& v1, const Class& v2, ChartJacobian H1 = boost::none,
      ChartJacobian H2 = boost::none) {
    if (H1) *H1 = - Jacobian::Identity();
    if (H2) *H2 =   Jacobian::Identity();
    return v2 - v1;
  }

  static Class Inverse(const Class& v, ChartJacobian H = boost::none) {
    if (H) *H = - Jacobian::Identity();
    return -v;
  }

  /// @}
};

/// VectorSpaceTraits implementation for dynamic types.
template<class Class>
struct VectorSpaceImpl<Class,Eigen::Dynamic> {

  /// @name Group
  /// @{
  static Class Compose(const Class& v1, const Class& v2) { return v1+v2;}
  static Class Between(const Class& v1, const Class& v2) { return v2-v1;}
  static Class Inverse(const Class& m) { return -m;}
  /// @}

  /// @name Manifold
  /// @{
  typedef Eigen::VectorXd TangentVector;
  typedef OptionalJacobian<Eigen::Dynamic,Eigen::Dynamic> ChartJacobian;
  static int GetDimension(const Class& m) { return m.dim();}

  static Eigen::MatrixXd Eye(const Class& m) {
    int dim = GetDimension(m);
    return Eigen::MatrixXd::Identity(dim, dim);
  }

  static TangentVector Local(const Class& origin, const Class& other,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = - Eye(origin);
    if (H2) *H2 = Eye(other);
    Class v = other-origin;
    return v.vector();
  }

  static Class Retract(const Class& origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(origin);
    if (H2) *H2 = Eye(origin);
    return origin + v;
  }

  /// @}

  /// @name Lie Group
  /// @{

  static TangentVector Logmap(const Class& m, ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Eye(m);
    return m.vector();
  }

  static Class Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    Class result(v);
    if (Hv)
      *Hv = Eye(v);
    return result;
  }

  static Class Compose(const Class& v1, const Class& v2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(v1);
    if (H2) *H2 = Eye(v2);
    return v1 + v2;
  }

  static Class Between(const Class& v1, const Class& v2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) *H1 = - Eye(v1);
    if (H2) *H2 =   Eye(v2);
    return v2 - v1;
  }

  static Class Inverse(const Class& v, ChartJacobian H) {
    if (H) *H = -Eye(v);
    return -v;
  }

  /// @}
};

/// Requirements on type to pass it to Manifold template below
template<class Class>
struct HasVectorSpacePrereqs {

  enum { dim = Class::dimension };

  Class p, q;
  Vector v;

  BOOST_CONCEPT_USAGE(HasVectorSpacePrereqs) {
    p = Class::identity();  // identity
    q = p + p;              // addition
    q = p - p;              // subtraction
    v = p.vector();         // conversion to vector
    q = p + v;              // addition of a vector on the right
  }
};

/// A helper that implements the traits interface for *classes* that define vector spaces
/// To use this for your class, define:
/// template<> struct traits<Class> : public VectorSpaceTraits<Class> {};
/// The class needs to support the requirements defined by HasVectorSpacePrereqs above
template<class Class>
struct VectorSpaceTraits: VectorSpaceImpl<Class, Class::dimension> {

  // Check that Class has the necessary machinery
  BOOST_CONCEPT_ASSERT((HasVectorSpacePrereqs<Class>));

  typedef vector_space_tag structure_category;

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static Class Identity() { return Class::identity();}
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = Class::dimension};
  typedef Class ManifoldType;
  /// @}
};

/// VectorSpace provides both Testable and VectorSpaceTraits
template<class Class>
struct VectorSpace: Testable<Class>, VectorSpaceTraits<Class> {};

/// A helper that implements the traits interface for scalar vector spaces. Usage:
/// template<> struct traits<Type> : public ScalarTraits<Type> { };
template<typename Scalar>
struct ScalarTraits : VectorSpaceImpl<Scalar, 1> {

  typedef vector_space_tag structure_category;

  /// @name Testable
  /// @{
  static void Print(Scalar m, const std::string& str = "") {
    gtsam::print(m, str);
  }
  static bool Equals(Scalar v1, Scalar v2, double tol = 1e-8) {
    return std::abs(v1 - v2) < tol;
  }
  /// @}

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static Scalar Identity() { return 0;}
  /// @}

  /// @name Manifold
  /// @{
  typedef Scalar ManifoldType;
  enum { dimension = 1 };
  typedef Eigen::Matrix<double, 1, 1> TangentVector;
  typedef OptionalJacobian<1, 1> ChartJacobian;

  static TangentVector Local(Scalar origin, Scalar other,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = -1.0;
    if (H2) (*H2)[0] =  1.0;
    TangentVector result;
    result(0) = other - origin;
    return result;
  }

  static Scalar Retract(Scalar origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = 1.0;
    if (H2) (*H2)[0] = 1.0;
    return origin + v[0];
  }
  /// @}

  /// @name Lie Group
  /// @{
  static TangentVector Logmap(Scalar m, ChartJacobian H = boost::none) {
    if (H) (*H)[0] = 1.0;
    return Local(0, m);
  }

  static Scalar Expmap(const TangentVector& v, ChartJacobian H = boost::none) {
    if (H) (*H)[0] = 1.0;
    return v[0];
  }
  /// @}

};

} // namespace internal

/// double
template<> struct traits<double> : public internal::ScalarTraits<double> {
};

/// float
template<> struct traits<float> : public internal::ScalarTraits<float> {
};

// traits for any fixed double Eigen matrix
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > :
    internal::VectorSpaceImpl<
        Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols>, M * N> {

  typedef vector_space_tag structure_category;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> Fixed;

  /// @name Testable
  /// @{
  static void Print(const Fixed& m, const std::string& str = "") {
    gtsam::print(Eigen::MatrixXd(m), str);
  }
  static bool Equals(const Fixed& v1, const Fixed& v2, double tol = 1e-8) {
    return equal_with_abs_tol(v1, v2, tol);
  }
  /// @}

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static Fixed Identity() { return Fixed::Zero();}
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = M*N};
  typedef Fixed ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  static TangentVector Local(const Fixed& origin, const Fixed& other,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) (*H1) = -Jacobian::Identity();
    if (H2) (*H2) =  Jacobian::Identity();
    TangentVector result;
    Eigen::Map<Fixed>(result.data()) = other - origin;
    return result;
  }

  static Fixed Retract(const Fixed& origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) (*H1) = Jacobian::Identity();
    if (H2) (*H2) = Jacobian::Identity();
    return origin + Eigen::Map<const Fixed>(v.data());
  }
  /// @}

  /// @name Lie Group
  /// @{
  static TangentVector Logmap(const Fixed& m, ChartJacobian H = boost::none) {
    if (H) *H = Jacobian::Identity();
    TangentVector result;
    Eigen::Map<Fixed>(result.data()) = m;
    return result;
  }

  static Fixed Expmap(const TangentVector& v, ChartJacobian H = boost::none) {
    Fixed m;
    m.setZero();
    if (H) *H = Jacobian::Identity();
    return m + Eigen::Map<const Fixed>(v.data());
  }
  /// @}
};


namespace internal {

// traits for dynamic Eigen matrices
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct DynamicTraits {

  typedef vector_space_tag structure_category;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> Dynamic;

  /// @name Testable
  /// @{
  static void Print(const Dynamic& m, const std::string& str = "") {
    gtsam::print(Eigen::MatrixXd(m), str);
  }
  static bool Equals(const Dynamic& v1, const Dynamic& v2,
      double tol = 1e-8) {
    return equal_with_abs_tol(v1, v2, tol);
  }
  /// @}

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static Dynamic Identity() {
    throw std::runtime_error("Identity not defined for dynamic types");
  }
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = Eigen::Dynamic };
  typedef Eigen::VectorXd TangentVector;
  typedef Eigen::MatrixXd Jacobian;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  typedef Dynamic ManifoldType;

  static int GetDimension(const Dynamic& m) {
    return m.rows() * m.cols();
  }

  static Jacobian Eye(const Dynamic& m) {
    int dim = GetDimension(m);
    return Eigen::Matrix<double, dimension, dimension>::Identity(dim, dim);
  }

  static TangentVector Local(const Dynamic& m, const Dynamic& other, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Eye(m);
    if (H2) *H2 =  Eye(m);
    TangentVector v(GetDimension(m));
    Eigen::Map<Dynamic>(v.data(), m.rows(), m.cols()) = other - m;
    return v;
  }

  static Dynamic Retract(const Dynamic& m, const TangentVector& v, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(m);
    if (H2) *H2 = Eye(m);
    return m + Eigen::Map<const Dynamic>(v.data(), m.rows(), m.cols());
  }
  /// @}

  /// @name Lie Group
  /// @{
  static TangentVector Logmap(const Dynamic& m, ChartJacobian H = boost::none) {
    if (H) *H = Eye(m);
    TangentVector result(GetDimension(m));
    Eigen::Map<Dynamic>(result.data(), m.cols(), m.rows()) = m;
    return result;
  }

  static Dynamic Expmap(const TangentVector& /*v*/, ChartJacobian H = boost::none) {
    static_cast<void>(H);
    throw std::runtime_error("Expmap not defined for dynamic types");
  }

  static Dynamic Inverse(const Dynamic& m, ChartJacobian H = boost::none) {
    if (H) *H = -Eye(m);
    return -m;
  }

  static Dynamic Compose(const Dynamic& v1, const Dynamic& v2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(v1);
    if (H2) *H2 = Eye(v1);
    return v1 + v2;
  }

  static Dynamic Between(const Dynamic& v1, const Dynamic& v2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Eye(v1);
    if (H2) *H2 = Eye(v1);
    return v2 - v1;
  }
  /// @}

};

} // \ internal

// traits for fully dynamic matrix
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<double, -1, -1, Options, MaxRows, MaxCols> > :
    public internal::DynamicTraits<-1, -1, Options, MaxRows, MaxCols> {
};

// traits for dynamic column vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<double, -1, 1, Options, MaxRows, MaxCols> > :
    public internal::DynamicTraits<-1, 1, Options, MaxRows, MaxCols> {
};

// traits for dynamic row vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<double, 1, -1, Options, MaxRows, MaxCols> > :
    public internal::DynamicTraits<1, -1, Options, MaxRows, MaxCols> {
};

/// Vector Space concept
template<typename T>
class IsVectorSpace: public IsLieGroup<T> {
public:

  typedef typename traits<T>::structure_category structure_category_tag;

  BOOST_CONCEPT_USAGE(IsVectorSpace) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<vector_space_tag, structure_category_tag>::value),
        "This type's trait does not assert it as a vector space (or derived)");
    r = p + q;
    r = -p;
    r = p - q;
  }

private:
  T p, q, r;
};

} // namespace gtsam

