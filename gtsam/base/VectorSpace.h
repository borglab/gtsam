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

template<typename T> struct traits_x;

namespace internal {

/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public VectorSpace<Type> { };
template<typename V>
struct VectorSpace: Testable<V> {

  typedef vector_space_tag structure_category;

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static V Identity() { return V::identity();}
  static V Compose(const V& v1, const V& v2) { return v1+v2;}
  static V Between(const V& v1, const V& v2) { return v2-v1;}
  static V Inverse(const V& m) { return -m;}
  /// @}

  /// @name Manifold
  /// @{
  enum {
    dimension = V::dimension
  };
  typedef V ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  static int GetDimension(const V& m) { return m.dim();}

  // TODO make more efficient in case of fied size
  static Eigen::Matrix<double, dimension, dimension> Eye(const V& m) {
    int dim = GetDimension(m);
    return Eigen::Matrix<double, dimension, dimension>::Identity(dim, dim);
  }

  static TangentVector Local(const V& origin, const V& other,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = - Eye(origin);
    if (H2) *H2 = Eye(other);
    V v = other-origin;
    return v.vector();
  }

  static V Retract(const V& origin, const TangentVector& v,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(origin);
    if (H2) *H2 = Eye(origin);
    return origin + V(v);
  }

  /// @}

  /// @name Lie Group
  /// @{

  static TangentVector Logmap(const V& m, ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Eye(m);
    return m.vector();
  }

  static V Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    if (Hv) {
      ManifoldType m;
      *Hv = Eye(m);
    }
    return V(v);
  }

  static V Compose(const V& v1, const V& v2, ChartJacobian H1,
      ChartJacobian H2) {
    if (H1) *H1 = Eye(v1);
    if (H2) *H2 = Eye(v2);
    return v1 + v2;
  }

  static V Between(const V& v1, const V& v2, ChartJacobian H1,
      ChartJacobian H2) {
    if (H1) *H1 = - Eye(v1);
    if (H2) *H2 =   Eye(v2);
    return v2 - v1;
  }

  static V Inverse(const V& v, ChartJacobian H) {
    if (H) *H = -Eye(v);
    return -v;
  }

  /// @}
};

/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public ScalarTraits<Type> { };
template<typename Scalar>
struct ScalarTraits {

  typedef vector_space_tag structure_category;

  /// @name Group
  /// @{
  typedef additive_group_tag group_flavor;
  static Scalar Identity() { return 0;}
  static Scalar Compose(const Scalar& v1, const Scalar& v2) { return v1+v2;}
  static Scalar Between(const Scalar& v1, const Scalar& v2) { return v2-v1;}
  static Scalar Inverse(const Scalar& m) { return -m;}
  /// @}

  // Typedefs required by all manifold types.
  typedef Scalar ManifoldType;
  enum { dimension = 1 };
  typedef Eigen::Matrix<double, 1, 1> TangentVector;
  typedef OptionalJacobian<1, 1> ChartJacobian;

  // For Testable
  static void Print(Scalar m, const std::string& str = "") {
    gtsam::print(m, str);
  }
  static bool Equals(Scalar v1, Scalar v2, double tol = 1e-8) {
    return fabs(v1 - v2) < tol;
  }

  static TangentVector Local(Scalar origin, Scalar other) {
    TangentVector result;
    result(0) = other - origin;
    return result;
  }

  static Scalar Retract(Scalar origin, const TangentVector& v) {
    return origin + v[0];
  }

  static TangentVector Local(Scalar origin, Scalar other, ChartJacobian Horigin,
      ChartJacobian Hother = boost::none) {
    if (Horigin) (*Horigin)[0] = -1.0;
    if (Hother) (*Hother)[0] = 1.0;
    return Local(origin, other);
  }

  static Scalar Retract(Scalar origin, const TangentVector& v,
      ChartJacobian Horigin, ChartJacobian Hv = boost::none) {
    if (Horigin) (*Horigin)[0] = 1.0;
    if (Hv) (*Hv)[0] = 1.0;
    return origin + v[0];
  }

  static int GetDimension(Scalar m) {return 1;}

  static Scalar Compose(Scalar v1, Scalar v2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = 1.0;
    if (H2) (*H2)[0] = 1.0;
    return v1 + v2;
  }

  static Scalar Between(Scalar v1, Scalar v2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = -1.0;
    if (H2) (*H2)[0] = 1.0;
    return v2 - v1;
  }

  static Scalar Inverse(Scalar m, ChartJacobian H) {
    if (H) (*H)[0] = -1;
    return -m;
  }

  static TangentVector Logmap(Scalar m) {
    return Local(0, m);
  }
  static Scalar Expmap(const TangentVector& v) {
    return v[0];
  }

  static TangentVector Logmap(Scalar m, ChartJacobian Hm) {
    if (Hm)
      (*Hm)[0] = 1.0;
    return Local(0, m);
  }

  static Scalar Expmap(const TangentVector& v, ChartJacobian Hv) {
    if (Hv)
      (*Hv)[0] = 1.0;
    return v[0];
  }

};

} // namespace internal

/// double
template<> struct traits_x<double> : public internal::ScalarTraits<double> {
};

/// float
template<> struct traits_x<float> : public internal::ScalarTraits<float> {
};

// traits for any double Eigen matrix
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits_x<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > {
//  BOOST_STATIC_ASSERT_MSG(
//      M != Eigen::Dynamic && N != Eigen::Dynamic,
//      "These traits are only valid on fixed-size types.");

  // Typedefs required by all manifold types.
  typedef vector_space_tag structure_category;
  enum {
    dimension = (
        M == Eigen::Dynamic ? Eigen::Dynamic :
            (N == Eigen::Dynamic ? Eigen::Dynamic : M * N))
  };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> ManifoldType;

  static int GetDimension(const ManifoldType& m) {
    return m.rows() * m.cols();
  }

  static Eigen::Matrix<double, dimension, dimension> Eye(
      const ManifoldType& m) {
    int dim = GetDimension(m);
    return Eigen::Matrix<double, dimension, dimension>::Identity(dim, dim);
  }

  // For Testable
  static void Print(const ManifoldType& m, const std::string& str = "") {
    gtsam::print(Eigen::MatrixXd(m), str);
  }
  static bool Equals(const ManifoldType& v1, const ManifoldType& v2,
      double tol = 1e-8) {
    return equal_with_abs_tol(v1, v2, 1e-9);
  }

  static TangentVector Local(const ManifoldType& origin,
      const ManifoldType& other, ChartJacobian Horigin = boost::none,
      ChartJacobian Hother = boost::none) {
    if (Horigin) *Horigin = -Eye(origin);
    if (Hother) *Hother = Eye(origin);
    TangentVector result(GetDimension(origin));
    Eigen::Map<Eigen::Matrix<double, M, N> >(result.data(), origin.rows(),
        origin.cols()) = other - origin;
    return result;
  }

  static ManifoldType Retract(const ManifoldType& origin,
      const TangentVector& v, ChartJacobian Horigin = boost::none,
      ChartJacobian Hv = boost::none) {
    if (Horigin) *Horigin = Eye(origin);
    if (Hv) *Hv = Eye(origin);
    return origin
        + Eigen::Map<const Eigen::Matrix<double, M, N> >(v.data(),
            origin.rows(), origin.cols());
  }

  static ManifoldType Compose(const ManifoldType& v1, const ManifoldType& v2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(v1);
    if (H2) *H2 = Eye(v1);
    return v1 + v2;
  }

  static ManifoldType Between(const ManifoldType& v1, const ManifoldType& v2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Eye(v1);
    if (H2) *H2 = Eye(v1);
    return v2 - v1;
  }

  static ManifoldType Inverse(const ManifoldType& m, ChartJacobian H =
      boost::none) {
    if (H) *H = -Eye(m);
    return -m;
  }

  static ManifoldType Identity() {
    //FIXME: this won't work for dynamic matrices, but where to get the size???
    return ManifoldType::Zero();
  }

  static TangentVector Logmap(const ManifoldType& m, ChartJacobian Hm =
      boost::none) {
    if (Hm)
      *Hm = Eye(m);
    TangentVector result(GetDimension(m));
    Eigen::Map<Eigen::Matrix<double, M, N> >(result.data()) = m;
    return result;
  }

  //FIXME: this also does not work for dynamic matrices
  static ManifoldType Expmap(const TangentVector& v, ChartJacobian Hv =
      boost::none) {
    ManifoldType m;
    m.setZero();
    if (Hv)
      *Hv = Eye(m);
    return m + Eigen::Map<Eigen::Matrix<double, M, N> >(v.data());
  }

};

/// Vector Space concept
template<typename V>
class IsVectorSpace: public IsLieGroup<V> {
public:

  typedef typename traits_x<V>::structure_category structure_category_tag;

  BOOST_CONCEPT_USAGE(IsVectorSpace) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<vector_space_tag, structure_category_tag>::value),
        "This type's trait does not assert it as a vector space (or derived)");
    r = p + q;
    r = -p;
    r = p - q;
  }

private:
  V p, q, r;
};

} // namespace gtsam

