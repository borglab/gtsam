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
struct vector_space_tag: public lie_group_tag {};

template <typename T> struct traits_x;

namespace internal {

/// A helper that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Type> : public ScalarTraits<Type> { };
template<typename Scalar>
struct ScalarTraits {
  // Typedefs required by all manifold types.
  typedef vector_space_tag structure_category;
  typedef additive_group_tag group_flavor;
  typedef Scalar ManifoldType;
  enum { dimension = 1 };
  typedef Eigen::Matrix<double, 1, 1> TangentVector;
  typedef OptionalJacobian<1, 1> ChartJacobian;

  // For Testable
  static void Print(Scalar m, const std::string& str = "") {
    gtsam::print(m, str);
  }
  static bool Equals(Scalar m1, Scalar m2, double tol = 1e-8) {
    return fabs(m1 - m2) < tol;
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
    return Local(origin,other);
  }

  static Scalar Retract(Scalar origin, const TangentVector& v,
      ChartJacobian Horigin, ChartJacobian Hv = boost::none) {
    if (Horigin) (*Horigin)[0] = 1.0;
    if (Hv) (*Hv)[0] = 1.0;
    return origin + v[0];
  }

  static int GetDimension(Scalar m) { return 1; }

  // For Group. Only implemented for groups
  static Scalar Compose(Scalar m1, Scalar m2) { return m1 + m2;}
  static Scalar Between(Scalar m1, Scalar m2) { return m2 - m1;}
  static Scalar Inverse(Scalar m) { return -m;}

  static Scalar Compose(Scalar m1, Scalar m2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = 1.0;
    if (H2) (*H2)[0] = 1.0;
    return m1 + m2;
  }

  static Scalar Between(Scalar m1, Scalar m2, ChartJacobian H1,
      ChartJacobian H2 = boost::none) {
    if (H1) (*H1)[0] = -1.0;
    if (H2) (*H2)[0] = 1.0;
    return m2 - m1;
  }

  static Scalar Inverse(Scalar m, ChartJacobian H) {
    if (H) (*H)[0] = -1;
    return -m;
  }

  static Scalar Identity() { return 0; }
  static TangentVector Logmap(Scalar m) {return Local(0,m);}
  static Scalar Expmap(const TangentVector& v) { return v[0];}

  static TangentVector Logmap(Scalar m, ChartJacobian Hm) {
    if (Hm) (*Hm)[0] = 1.0;
    return Local(0,m);
  }

  static Scalar Expmap(const TangentVector& v, ChartJacobian Hv) {
    if (Hv) (*Hv)[0] = 1.0;
    return v[0];
  }

};

}  // namespace internal

/// double
template<> struct traits_x<double> : public internal::ScalarTraits<double> {};

/// float
template<> struct traits_x<float> : public internal::ScalarTraits<float> {};

// traits for any double Eigen matrix
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits_x< Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> > {
//  BOOST_STATIC_ASSERT_MSG(
//      M != Eigen::Dynamic && N != Eigen::Dynamic,
//      "These traits are only valid on fixed-size types.");

  // Typedefs required by all manifold types.
  typedef vector_space_tag structure_category;
  enum { dimension = (M == Eigen::Dynamic ? Eigen::Dynamic :
              (N == Eigen::Dynamic ? Eigen::Dynamic : M * N)) };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> ManifoldType;

  static int GetDimension(const ManifoldType& m){ return m.rows()*m.cols(); }

  static Eigen::Matrix<double, dimension, dimension> Eye(const ManifoldType& m) {
    int dim = GetDimension(m);
    return Eigen::Matrix<double, dimension, dimension>::Identity(dim,dim);
  }

  // For Testable
  static void Print(const ManifoldType& m, const std::string& str = "") {
    gtsam::print(Eigen::MatrixXd(m), str);
  }
  static bool Equals(const ManifoldType& m1, const ManifoldType& m2,
              double tol = 1e-8) {
    return equal_with_abs_tol(m1, m2, 1e-9);
  }

  static TangentVector Local(const ManifoldType& origin,
                             const ManifoldType& other,
                             ChartJacobian Horigin = boost::none,
                             ChartJacobian Hother = boost::none) {
   if (Horigin) *Horigin = -Eye(origin);
   if (Hother) *Hother = Eye(origin);
   TangentVector result(GetDimension(origin));
   Eigen::Map<Eigen::Matrix<double, M, N> >(
    result.data(), origin.rows(), origin.cols()) = other - origin;
   return result;
  }

  static ManifoldType Retract(const ManifoldType& origin,
                              const TangentVector& v,
                              ChartJacobian Horigin = boost::none,
                              ChartJacobian Hv = boost::none) {
    if (Horigin) *Horigin = Eye(origin);
    if (Hv) *Hv = Eye(origin);
    return origin + Eigen::Map<const Eigen::Matrix<double, M, N> >(v.data(), origin.rows(), origin.cols());
  }

  static ManifoldType Compose(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1 = boost::none,
                              ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Eye(m1);
    if (H2) *H2 = Eye(m1);
    return m1+m2;
  }

  static ManifoldType Between(const ManifoldType& m1,
                              const ManifoldType& m2,
                              ChartJacobian H1 = boost::none,
                              ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Eye(m1);
    if (H2) *H2 = Eye(m1);
    return m2-m1;
  }

  static ManifoldType Inverse(const ManifoldType& m,
                              ChartJacobian H = boost::none) {
    if (H) *H = -Eye(m);
    return -m;
  }

  static ManifoldType Identity() {
    //FIXME: this won't work for dynamic matrices, but where to get the size???
    return ManifoldType::Zero();
  }

  static TangentVector Logmap(const ManifoldType& m, ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Eye(m);
    TangentVector result(GetDimension(m));
    Eigen::Map<Eigen::Matrix<double, M, N> >(
      result.data()) = m;
    return result;
  }

  //FIXME: this also does not work for dynamic matrices
  static ManifoldType Expmap(const TangentVector& v, ChartJacobian Hv = boost::none) {
    ManifoldType m; m.setZero();
    if (Hv) *Hv = Eye(m);
    return m +
        Eigen::Map<Eigen::Matrix<double, M, N> >(v.data());
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

