/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Lie.h
 * @brief Base class and basic functions for Lie types
 * @author Richard Roberts
 * @author Alex Cunningham
 * @author Frank Dellaert
 * @author Mike Bosse
 * @author Duy Nguyen Ta
 * @author Yotam Stern
 */


#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Group.h>

#include <type_traits>

namespace gtsam {

/// A CRTP helper class that implements Lie group methods
/// Prerequisites: methods operator*, inverse, and AdjointMap, as well as a
/// ChartAtOrigin struct that will be used to define the manifold Chart
/// To use, simply derive, but also say "using LieGroup<Class,N>::inverse"
/// For derivative math, see doc/math.pdf
template <class Class, int N>
struct LieGroup {

  inline constexpr static auto dimension = N;
  typedef OptionalJacobian<N, N> ChartJacobian;
  typedef Eigen::Matrix<double, N, N> Jacobian;
  typedef Eigen::Matrix<double, N, 1> TangentVector;

  /// Static method to get the dimension (compile-time or dynamic)
  static constexpr int Dim() { return N; }

  /// Provided fixed dimension in dim() if needed
  template <int M = N>
  std::enable_if_t<M != Eigen::Dynamic, int> dim() const {
    return N;
  }

  const Class & derived() const {
    return static_cast<const Class&>(*this);
  }

  Class compose(const Class& g) const {
    return derived() * g;
  }

  Class between(const Class& g) const {
    return derived().inverse() * g;
  }

  Class compose(const Class& g, ChartJacobian H1,
      ChartJacobian H2 = {}) const {
    if (H1) *H1 = g.inverse().AdjointMap();
    if (H2) *H2 = identityMatrix();
    return derived() * g;
  }

  Class between(const Class& g, ChartJacobian H1,
      ChartJacobian H2 = {}) const {
    Class result = derived().inverse() * g;
    if (H1) *H1 = - result.inverse().AdjointMap();
    if (H2) *H2 = identityMatrix();
    return result;
  }

  Class inverse(ChartJacobian H) const {
    if (H) *H = - derived().AdjointMap();
    return derived().inverse();
  }

  /// expmap as required by manifold concept
  /// Applies exponential map to v and composes with *this
  Class expmap(const TangentVector& v) const {
    return compose(Class::Expmap(v));
  }

  /// logmap as required by manifold concept
  /// Applies logarithmic map to group element that takes *this to g
  TangentVector logmap(const Class& g) const {
    return Class::Logmap(between(g));
  }

  /// expmap with optional derivatives
  Class expmap(const TangentVector& v, //
      ChartJacobian H1, ChartJacobian H2 = {}) const {
    Jacobian D_g_v;
    Class g = Class::Expmap(v,H2 ? &D_g_v : 0);
    Class h = compose(g); // derivatives inlined below
    if (H1) *H1 = g.inverse().AdjointMap();
    if (H2) *H2 = D_g_v;
    return h;
  }

  /// logmap with optional derivatives
  TangentVector logmap(const Class& g, //
      ChartJacobian H1, ChartJacobian H2 = {}) const {
    Class h = between(g); // derivatives inlined below
    Jacobian D_v_h;
    TangentVector v = Class::Logmap(h, (H1 || H2) ? &D_v_h : 0);
    if (H1) *H1 = - D_v_h * h.inverse().AdjointMap();
    if (H2) *H2 = D_v_h;
    return v;
  }

  /// Retract at origin: possible in Lie group because it has an identity
  static Class Retract(const TangentVector& v) {
    return Class::ChartAtOrigin::Retract(v);
  }

  /// LocalCoordinates at origin: possible in Lie group because it has an identity
  static TangentVector LocalCoordinates(const Class& g) {
    return Class::ChartAtOrigin::Local(g);
  }

  /// Retract at origin with optional derivative
  static Class Retract(const TangentVector& v, ChartJacobian H) {
    return Class::ChartAtOrigin::Retract(v,H);
  }

  /// LocalCoordinates at origin with optional derivative
  static TangentVector LocalCoordinates(const Class& g, ChartJacobian H) {
    return Class::ChartAtOrigin::Local(g,H);
  }

  /// retract as required by manifold concept: applies v at *this
  Class retract(const TangentVector& v) const {
    return compose(Class::ChartAtOrigin::Retract(v));
  }

  /// localCoordinates as required by manifold concept: finds tangent vector between *this and g
  TangentVector localCoordinates(const Class& g) const {
    return Class::ChartAtOrigin::Local(between(g));
  }

  /// retract with optional derivatives
  Class retract(const TangentVector& v, //
      ChartJacobian H1, ChartJacobian H2 = {}) const {
    Jacobian D_g_v;
    Class g = Class::ChartAtOrigin::Retract(v, H2 ? &D_g_v : 0);
    Class h = compose(g); // derivatives inlined below
    if (H1) *H1 = g.inverse().AdjointMap();
    if (H2) *H2 = D_g_v;
    return h;
  }

  /// localCoordinates with optional derivatives
  TangentVector localCoordinates(const Class& g, //
      ChartJacobian H1, ChartJacobian H2 = {}) const {
    Class h = between(g); // derivatives inlined below
    Jacobian D_v_h;
    TangentVector v = Class::ChartAtOrigin::Local(h, (H1 || H2) ? &D_v_h : 0);
    if (H1) *H1 = - D_v_h * h.inverse().AdjointMap();
    if (H2) *H2 = D_v_h;
    return v;
  }

 private:

  // Helper to get identity matrix of correct size for static or dynamic N
  Jacobian identityMatrix() const {
    if constexpr (N == Eigen::Dynamic) {
      return Jacobian::Identity(derived().dim(), derived().dim());
    } else {
      return Jacobian::Identity();
    }
  }
};

/// tag to assert a type is a Lie group
struct lie_group_tag: public manifold_tag, public group_tag {};

namespace internal {

/// A helper class that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Class> : public internal::LieGroupTraits<Class> {};
/// Assumes existence of: identity, dimension, localCoordinates, retract,
/// and additionally Logmap, Expmap, AdjointMap, compose, between, and inverse
template<class Class>
struct LieGroupTraits : public GetDimensionImpl<Class, Class::dimension> {
  using structure_category = lie_group_tag;

  /// @name Group
  /// @{
  using group_flavor = multiplicative_group_tag;
  static Class Identity() { return Class::Identity(); }
  /// @}

  /// @name Manifold
  /// @{
  using ManifoldType = Class;
  // Note: Class::dimension can be an int or Eigen::Dynamic.
  // GetDimensionImpl handles resolving this to a static value or providing GetDimension(obj).
  inline constexpr static auto dimension = Class::dimension;
  using TangentVector = Eigen::Matrix<double, dimension, 1>;
  using ChartJacobian = OptionalJacobian<dimension, dimension>;

  static TangentVector Local(const Class& origin, const Class& other,
    ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    return origin.localCoordinates(other, H1, H2);
  }

  static Class Retract(const Class& origin, const TangentVector& v,
    ChartJacobian H = {}, ChartJacobian Hv = {}) {
    return origin.retract(v, H, Hv);
  }
  /// @}

  /// @name Lie Group
  /// @{
  static TangentVector Logmap(const Class& m, ChartJacobian Hm = {}) {
    return Class::Logmap(m, Hm);
  }

  static Class Expmap(const TangentVector& v, ChartJacobian Hv = {}) {
    return Class::Expmap(v, Hv);
  }

  static Class Compose(const Class& m1, const Class& m2, //
    ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    return m1.compose(m2, H1, H2);
  }

  static Class Between(const Class& m1, const Class& m2, //
    ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    return m1.between(m2, H1, H2);
  }

  static Class Inverse(const Class& m, //
    ChartJacobian H = {}) {
    return m.inverse(H);
  }

  static Eigen::Matrix<double, dimension, dimension> AdjointMap(const Class& m) {
    // This assumes that the Class itself provides a member function `AdjointMap()`
    // For dynamically-sized types (dimension == Eigen::Dynamic),
    // m.AdjointMap() must return a gtsam::Matrix of the correct runtime dimensions.
    return m.AdjointMap();
  }
  /// @}
};


/// Both LieGroupTraits and Testable
template<class Class> struct LieGroup: LieGroupTraits<Class>, Testable<Class> {};

} // \ namespace internal

/**
 * These core global functions can be specialized by new Lie types
 * for better performance.
 */

/** Compute l0 s.t. l2=l1*l0 */
template<class Class>
inline Class between_default(const Class& l1, const Class& l2) {
  return l1.inverse().compose(l2);
}

/** Log map centered at l0, s.t. exp(l0,log(l0,lp)) = lp */
template<class Class>
inline Vector logmap_default(const Class& l0, const Class& lp) {
  return Class::Logmap(l0.between(lp));
}

/** Exponential map centered at l0, s.t. exp(t,d) = t*exp(d) */
template<class Class>
inline Class expmap_default(const Class& t, const Vector& d) {
  return t.compose(Class::Expmap(d));
}

/**
 * Lie Group Concept
 */
template<typename T>
class IsLieGroup: public IsGroup<T>, public IsManifold<T> {
public:
  // Concept marker: allows checking IsLieGroup<T>::value in templates
  static constexpr bool value =
    std::is_base_of_v<lie_group_tag, typename traits<T>::structure_category>;

  using structure_category_tag = typename traits<T>::structure_category;
  using ManifoldType = typename traits<T>::ManifoldType;
  using TangentVector = typename traits<T>::TangentVector;
  using ChartJacobian = typename traits<T>::ChartJacobian;

  GTSAM_CONCEPT_USAGE(IsLieGroup) {
    static_assert(
        value,
        "This type's trait does not assert it is a Lie group (or derived)");

    // group operations with Jacobians
    g = traits<T>::Compose(g, h, Hg, Hh);
    g = traits<T>::Between(g, h, Hg, Hh);
    g = traits<T>::Inverse(g, Hg);
    // log and exp map without Jacobians
    g = traits<T>::Expmap(v);
    v = traits<T>::Logmap(g);
    // log and exponential map with Jacobians
    g = traits<T>::Expmap(v, Hg);
    v = traits<T>::Logmap(g, Hg);
    // AdjointMap
    *Hg = traits<T>::AdjointMap(g);
  }
private:
  T g, h;
  TangentVector v;
  ChartJacobian Hg, Hh;
};

/**
 * Linear interpolation between X and Y by coefficient t. Typically t \in [0,1],
 * but can also be used to extrapolate before pose X or after pose Y.
 */
template <typename T>
T interpolate(const T& X, const T& Y, double t,
              typename MakeOptionalJacobian<T, T>::type Hx = {},
              typename MakeOptionalJacobian<T, T>::type Hy = {},
              typename MakeOptionalJacobian<T, double>::type Ht = {}) {
  if (Hx || Hy || Ht) {
    typename MakeJacobian<T, T>::type between_H_x, log_H, exp_H, compose_H_x;
    const T between =
        traits<T>::Between(X, Y, between_H_x);  // between_H_y = identity
    typename traits<T>::TangentVector delta = traits<T>::Logmap(between, log_H);
    const T Delta = traits<T>::Expmap(t * delta, exp_H);
    const T result = traits<T>::Compose(
        X, Delta, compose_H_x);  // compose_H_xinv_y = identity

    if (Hx) *Hx = compose_H_x + t * exp_H * log_H * between_H_x;
    if (Hy) *Hy = t * exp_H * log_H;
    if (Ht) *Ht = delta;
    return result;
  }
  return traits<T>::Compose(
      X, traits<T>::Expmap(t * traits<T>::Logmap(traits<T>::Between(X, Y))));
}

/**
 * Functor for transforming covariance of T.
 * T needs to satisfy the Lie group concept.
 */
template<class T>
class TransformCovariance
{
private:
  typename T::Jacobian adjointMap_;
public:
  explicit TransformCovariance(const T &X) : adjointMap_{X.AdjointMap()} {}
  typename T::Jacobian operator()(const typename T::Jacobian &covariance)
  { return adjointMap_ * covariance * adjointMap_.transpose(); }
};

} // namespace gtsam

/**
 * Macros for using the LieConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_LIE_INST(T) template class gtsam::IsLieGroup<T>;
#define GTSAM_CONCEPT_LIE_TYPE(T) using _gtsam_IsLieGroup_##T = gtsam::IsLieGroup<T>;
