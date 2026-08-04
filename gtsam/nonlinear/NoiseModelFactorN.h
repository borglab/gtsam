/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NoiseModelFactorN.h
 * @brief   Base class for noise model factors with N variables
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Gerry Chen
 * @author  Fan Jiang
 */

#pragma once
// \callgraph

#include <gtsam/nonlinear/NonlinearFactor.h>
namespace gtsam {

/* ************************************************************************* */
namespace detail {
/** Convenience base class to add aliases `X1`, `X2`, ..., `X6` -> ValueType<N>.
 * Usage example:
 * ```
 * class MyFactor : public NoiseModelFactorT<Vector, Pose3, Point3>,
 *                  public NoiseModelFactorAliases<Pose3, Point3> {
 *  // class implementation ...
 * };
 *
 * // MyFactor::X1 == Pose3
 * // MyFactor::X2 == Point3
 * ```
 */
template <typename, typename...>
struct NoiseModelFactorAliases {};
template <typename T1>
struct NoiseModelFactorAliases<T1> {
  using X = T1;
  using X1 = T1;
};
template <typename T1, typename T2>
struct NoiseModelFactorAliases<T1, T2> {
  using X1 = T1;
  using X2 = T2;
};
template <typename T1, typename T2, typename T3>
struct NoiseModelFactorAliases<T1, T2, T3> {
  using X1 = T1;
  using X2 = T2;
  using X3 = T3;
};
template <typename T1, typename T2, typename T3, typename T4>
struct NoiseModelFactorAliases<T1, T2, T3, T4> {
  using X1 = T1;
  using X2 = T2;
  using X3 = T3;
  using X4 = T4;
};
template <typename T1, typename T2, typename T3, typename T4, typename T5>
struct NoiseModelFactorAliases<T1, T2, T3, T4, T5> {
  using X1 = T1;
  using X2 = T2;
  using X3 = T3;
  using X4 = T4;
  using X5 = T5;
};
template <typename T1, typename T2, typename T3, typename T4, typename T5,
          typename T6, typename... TExtra>
struct NoiseModelFactorAliases<T1, T2, T3, T4, T5, T6, TExtra...> {
  using X1 = T1;
  using X2 = T2;
  using X3 = T3;
  using X4 = T4;
  using X5 = T5;
  using X6 = T6;
};
}  // namespace detail

/* ************************************************************************* */
/**
 * A convenient base class for creating your own NoiseModelFactor
 * with n variables.  To derive from this class, implement evaluateError().
 *
 * For example, a 2-way factor that computes the difference in x-translation
 * between a Pose3 and Point3 could be implemented like so:
 *
 * ~~~~~~~~~~~~~~~~~~~~{.cpp}
 * class MyFactor : public NoiseModelFactorT<Vector, Pose3, Point3> {
 *  public:
 *   using Base = NoiseModelFactorT<Vector, Pose3, Point3>;
 *
 *   MyFactor(Key pose_key, Key point_key, const SharedNoiseModel& noiseModel)
 *       : Base(noiseModel, pose_key, point_key) {}
 *
 *   Vector evaluateError(
 *       const Pose3& T, const Point3& p,
 *       OptionalMatrixType H_T = OptionalNone,
 *       OptionalMatrixType H_p = OptionalNone) const override {
 *     Matrix36 t_H_T;  // partial derivative of translation w.r.t. pose T
 *
 *     // Only compute t_H_T if needed:
 *     Point3 t = T.translation(H_T ? &t_H_T : 0);
 *     double a = t(0); // a_H_t = [1, 0, 0]
 *     double b = p(0); // b_H_p = [1, 0, 0]
 *     double error = a - b; // H_a = 1, H_b = -1
 *
 *     // H_T = H_a * a_H_t * t_H_T = the first row of t_H_T
 *     if (H_T) *H_T = (Matrix(1, 6) << t_H_T.row(0)).finished();
 *     // H_p = H_b * b_H_p = -1 * [1, 0, 0]
 *     if (H_p) *H_p = (Matrix(1, 3) << -1., 0., 0.).finished();
 *
 *     return Vector1(error);
 *   }
 * };
 *
 * // Unit Test
 * TEST(NonlinearFactor, MyFactor) {
 *   MyFactor f(X(1), X(2), noiseModel::Unit::Create(1));
 *   EXPECT_DOUBLES_EQUAL(-8., f.evaluateError(Pose3(), Point3(8., 7., 6.))(0),
 *                        1e-9);
 *   Values values;
 *   values.insert(X(1), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1, 2, 3)));
 *   values.insert(X(2), Point3(1, 2, 3));
 *   EXPECT_CORRECT_FACTOR_JACOBIANS(f, values, 1e-5, 1e-5);
 * }
 * ~~~~~~~~~~~~~~~~~~~~
 *
 * These factors are templated on a values structure type. The values structures
 * are typically more general than just vectors, e.g., Rot3 or Pose3, which are
 * objects in non-linear manifolds (Lie groups).
 *
 * @tparam OutputVec The type of the error vector, usually Vector.
 * @tparam ValueTypes The types of the variables connected to this factor, e.g., Pose3, Point3.
 */
template <class OutputVec, class... ValueTypes>
class NoiseModelFactorT
    : public NoiseModelFactor,
      public detail::NoiseModelFactorAliases<ValueTypes...> {
 public:
  /// N is the number of variables (N-way factor)
  inline constexpr static auto N = sizeof...(ValueTypes);

  using NoiseModelFactor::unwhitenedError;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactorT<OutputVec, ValueTypes...>;

  /// @name SFINAE aliases
  /// @{

  template <typename From, typename To>
  using IsConvertible =
      typename std::enable_if<std::is_convertible<From, To>::value, void>::type;

  template <int I>
  using IndexIsValid = typename std::enable_if<(I >= 1) && (I <= N),
                                               void>::type;  // 1-indexed!

  template <typename Container>
  using ContainerElementType =
      typename std::decay<decltype(*std::declval<Container>().begin())>::type;
  template <typename Container>
  using IsContainerOfKeys = IsConvertible<ContainerElementType<Container>, Key>;

  /** A helper alias to check if a list of args
   * are all references to a matrix or not. It will be used
   * to choose the right overload of evaluateError.
   */
  template <typename Ret, typename... Args>
  using AreAllMatrixRefs =
      std::enable_if_t<(... && std::is_convertible<Args, Matrix&>::value), Ret>;

  template <typename Arg>
  using IsMatrixPointer = std::is_same<typename std::decay_t<Arg>, Matrix*>;

  template <typename Arg>
  using IsNullpointer =
      std::is_same<typename std::decay_t<Arg>, std::nullptr_t>;

  /** A helper alias to check if a list of args
   * are all pointers to a matrix or not. It will be used
   * to choose the right overload of evaluateError.
   */
  template <typename Ret, typename... Args>
  using AreAllMatrixPtrs =
      std::enable_if_t<(... && (IsMatrixPointer<Args>::value ||
                                IsNullpointer<Args>::value)),
                       Ret>;

  /// @}

  /* Like std::void_t, except produces `OptionalMatrixType` instead of
   * `void`. Used to expand fixed-type parameter-packs with same length as
   * ValueTypes. */
  template <typename T = void>
  using OptionalMatrixTypeT = Matrix*;

  /* Like std::void_t, except produces `Key` instead of `void`. Used to expand
   * fixed-type parameter-packs with same length as ValueTypes. */
  template <typename T>
  using KeyType = Key;

  /* Like std::void_t, except produces `Matrix` instead of
   * `void`. Used to expand fixed-type parameter-packs with same length as
   * ValueTypes. This helps in creating an evaluateError overload that accepts
   * Matrices instead of pointers to matrices */
  template <typename T = void>
  using MatrixTypeT = Matrix;

 public:
  /**
   * The type of the I'th template param can be obtained as ValueType<I>.
   * I is 1-indexed for backwards compatibility/consistency!  So for example,
   * ```
   * using Factor = NoiseModelFactorT<Vector, Pose3, Point3>;
   * Factor::ValueType<1>  // Pose3
   * Factor::ValueType<2>  // Point3
   * // Factor::ValueType<0> // ERROR!  Will not compile.
   * // Factor::ValueType<3> // ERROR!  Will not compile.
   * ```
   *
   * You can also use the shortcuts `X1`, ..., `X6` which are the same as
   * `ValueType<1>`, ..., `ValueType<6>` respectively (see
   * detail::NoiseModelFactorAliases).
   *
   * Note that, if your class is templated AND you want to use `ValueType<1>`
   * inside your class, due to dependent types you need the `template` keyword:
   * `typename MyFactor<T>::template ValueType<1>`.
   */
  template <int I, typename = IndexIsValid<I>>
  using ValueType =
      typename std::tuple_element<I - 1, std::tuple<ValueTypes...>>::type;

 public:
  /// @name Constructors
  /// @{

  /// Default Constructor for I/O
  NoiseModelFactorT() {}

  /**
   * Constructor.
   * Example usage: NoiseModelFactorT(noise, key1, key2, ..., keyN)
   * @param noiseModel Shared pointer to noise model.
   * @param keys Keys for the variables in this factor, passed in as separate
   * arguments.
   */
  NoiseModelFactorT(const SharedNoiseModel& noiseModel,
                    KeyType<ValueTypes>... keys)
      : Base(noiseModel, std::array<Key, N>{keys...}) {}

  /**
   * Constructor.
   * Example usage: `NoiseModelFactorT(noise, {key1, key2, ..., keyN})`
   * Example usage: `NoiseModelFactorT(noise, keys)` where keys is a
   * vector<Key>
   * @param noiseModel Shared pointer to noise model.
   * @param keys A container of keys for the variables in this factor.
   */
  template <typename CONTAINER = std::initializer_list<Key>,
            typename = IsContainerOfKeys<CONTAINER>>
  NoiseModelFactorT(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    if (keys.size() != N) {
      throw std::invalid_argument(
          "NoiseModelFactorT: wrong number of keys given");
    }
  }

  /// @}

  ~NoiseModelFactorT() override {}

  /** Returns a key. Usage: `key<I>()` returns the I'th key.
   * I is 1-indexed for backwards compatibility/consistency!  So for example,
   * ```
   * NoiseModelFactorT<Vector, Pose3, Point3> factor(noise, key1, key2);
   * key<1>()  // = key1
   * key<2>()  // = key2
   * // key<0>()  // ERROR!  Will not compile
   * // key<3>()  // ERROR!  Will not compile
   * ```
   *
   * Note that, if your class is templated AND you are trying to call `key<1>`
   * inside your class, due to dependent types you need the `template` keyword:
   * `this->key1()`.
   */
  template <int I = 1>
  inline Key key() const {
    static_assert(I <= N, "Index out of bounds");
    return keys_[I - 1];
  }

  /// @name NoiseModelFactor methods
  /// @{

  /** This implements the `unwhitenedError` virtual function by calling the
   * n-key specific version of evaluateError, which is pure virtual so must be
   * implemented in the derived class.
   *
   * Example usage:
   * ```
   *  gtsam::Values values;
   *  values.insert(...) // populate values
   *  std::vector<Matrix> Hs(2); // this will be an optional output argument
   *  const Vector error = factor.unwhitenedError(values, Hs);
   * ```
   * @param[in] x A Values object containing the values of all the variables
   * used in this factor
   * @param[out] H A vector of (dynamic) matrices whose size should be equal to
   * n.  The Jacobians w.r.t. each variable will be output in this parameter.
   */
  Vector unwhitenedError(const Values& x,
                         OptionalMatrixVecType H = nullptr) const override {
    return unwhitenedError(gtsam::index_sequence_for<ValueTypes...>{}, x, H);
  }

  /// @}
  /// @name Virtual methods
  /// @{

  /**
   * Override `evaluateError` to finish implementing an n-way factor.
   *
   * Both the `x` and `H` arguments are written here as parameter packs, but
   * when overriding this method, you probably want to explicitly write them
   * out.  For example, for a 2-way factor with variable types Pose3 and Point3,
   * you should implement:
   * ```
   * Vector evaluateError(
   *     const Pose3& x1, const Point3& x2,
   *     OptionalMatrixType H1 = OptionalNone,
   *     OptionalMatrixType H2 = OptionalNone) const override { ... }
   * ```
   *
   * If any of the optional Matrix reference arguments are specified, it should
   * compute both the function evaluation and its derivative(s) in the requested
   * variables.
   *
   * @param x The values of the variables to evaluate the error for.  Passed in
   * as separate arguments.
   * @param[out] H The Jacobian with respect to each variable (optional).
   */
  virtual OutputVec evaluateError(
      const ValueTypes&... x, OptionalMatrixTypeT<ValueTypes>... H) const = 0;

  /** If all the optional arguments are matrices then redirect the call to
   * the one which takes pointers.
   * To get access to this version of the function from derived classes
   * one will need to use the "using" keyword and specify that like this:
   * public:
   *   using NoiseModelFactorT<list the value types here>::evaluateError;
   */
  Vector evaluateError(const ValueTypes&... x,
                       MatrixTypeT<ValueTypes>&... H) const {
    return evaluateError(x..., (&H)...);
  }

  /// @}
  /// @name Convenience method overloads
  /// @{

  /** No-Jacobians requested function overload.
   * This specializes the version below to avoid recursive calls since this is
   * commonly used.
   *
   * e.g. `const Vector error = factor.evaluateError(pose, point);`
   */
  inline Vector evaluateError(const ValueTypes&... x) const {
    return evaluateError(x..., OptionalMatrixTypeT<ValueTypes>()...);
  }

  /** Some (but not all) optional Jacobians are omitted (function overload)
   * and the jacobians are l-value references to matrices.
   * e.g. `const Vector error = factor.evaluateError(pose, point, Hpose);`
   */
  template <typename... OptionalJacArgs,
            typename = IndexIsValid<sizeof...(OptionalJacArgs) + 1>>
  inline AreAllMatrixRefs<Vector, OptionalJacArgs...> evaluateError(
      const ValueTypes&... x, OptionalJacArgs&&... H) const {
    return evaluateError(x..., (&H)...);
  }

  /** Some (but not all) optional Jacobians are omitted (function overload)
   * and the jacobians are pointers to matrices.
   * e.g. `const Vector error = factor.evaluateError(pose, point, &Hpose);`
   */
  template <typename... OptionalJacArgs,
            typename = IndexIsValid<sizeof...(OptionalJacArgs) + 1>>
  inline AreAllMatrixPtrs<Vector, OptionalJacArgs...> evaluateError(
      const ValueTypes&... x, OptionalJacArgs&&... H) const {
    // If they are pointer version, ensure to cast them all to be Matrix* types
    // This will ensure any arguments inferred as std::nonetype_t are cast to
    // (Matrix*) nullptr This guides the compiler to the correct overload which
    // is the one that takes pointers
    return evaluateError(x..., std::forward<OptionalJacArgs>(H)...,
                         static_cast<OptionalMatrixType>(OptionalNone));
  }

  /// @}

 private:
  /** Pack expansion with index_sequence template pattern, used to index into
   * `keys_` and `H`.
   *
   * Example: For `NoiseModelFactorT<Vector, Pose3, Point3>`, the call would
   * look like: `const Vector error = unwhitenedError(0, 1, values, H);`
   */
  template <std::size_t... Indices>
  inline Vector unwhitenedError(gtsam::index_sequence<Indices...>,  //
                                const Values& x,
                                OptionalMatrixVecType H = nullptr) const {
    if (this->active(x)) {
      if (H) {
        return evaluateError(x.at<ValueTypes>(keys_[Indices])...,
                             (*H)[Indices]...);
      } else {
        return evaluateError(x.at<ValueTypes>(keys_[Indices])...);
      }
    } else {
      return Vector::Zero(this->dim());
    }
  }

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
#endif

 public:
  /// @name Shortcut functions `key1()` -> `key<1>()`
  /// @{

  inline Key key1() const { return key<1>(); }
  template <int I = 2>
  inline Key key2() const {
    static_assert(I <= N, "Index out of bounds");
    return key<2>();
  }
  template <int I = 3>
  inline Key key3() const {
    static_assert(I <= N, "Index out of bounds");
    return key<3>();
  }
  template <int I = 4>
  inline Key key4() const {
    static_assert(I <= N, "Index out of bounds");
    return key<4>();
  }
  template <int I = 5>
  inline Key key5() const {
    static_assert(I <= N, "Index out of bounds");
    return key<5>();
  }
  template <int I = 6>
  inline Key key6() const {
    static_assert(I <= N, "Index out of bounds");
    return key<6>();
  }

  /// @}

};  // \class NoiseModelFactorT

/**
 * @brief Noise model factor with N value types and dynamic-sized error vector
 *
 * @tparam ValueTypes
 */
template <class... ValueTypes>
using NoiseModelFactorN = NoiseModelFactorT<Vector, ValueTypes...>;

#define NoiseModelFactor1 NoiseModelFactorN
#define NoiseModelFactor2 NoiseModelFactorN
#define NoiseModelFactor3 NoiseModelFactorN
#define NoiseModelFactor4 NoiseModelFactorN
#define NoiseModelFactor5 NoiseModelFactorN
#define NoiseModelFactor6 NoiseModelFactorN
}  // namespace gtsam