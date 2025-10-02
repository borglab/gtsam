#pragma once

#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <tuple>
#include <type_traits>
#include <utility>

#include "gtsam/base/GenericValue.h"
#include "gtsam/base/utilities.h"

namespace gtsam {
/* ************************************************************************* */
/**
 * A convenient base class for creating your own NoiseModelFactor
 * with n variables.  To derive from this class, implement evaluateError().
 *
 * For example, a 2-way factor that computes the difference in x-translation
 * between a Pose3 and Point3 could be implemented like so:
 *
 * ~~~~~~~~~~~~~~~~~~~~{.cpp}
 * class MyFactor : public FastNoiseModelFactorN<Pose3, Point3> {
 *  public:
 *   using Base = FastNoiseModelFactorN<Pose3, Point3>;
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
 */
template <class... ValueTypes>
class FastNoiseModelFactorN
    : public NoiseModelFactor,
      public detail::NoiseModelFactorAliases<ValueTypes...> {
 public:
  /// N is the number of variables (N-way factor)
  inline constexpr static auto N = sizeof...(ValueTypes);

  using NoiseModelFactor::unwhitenedError;

 protected:
  using Base = NoiseModelFactor;
  using This = FastNoiseModelFactorN<ValueTypes...>;

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

  /* Like std::void_t, except produces `Eigen::Ref<Matrix>` instead of `void`.
   */
  template <typename T = void>
  using OptionalMatrixRefTypeT = Eigen::Ref<Matrix>*;

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
   * using Factor = FastNoiseModelFactorN<Pose3, Point3>;
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
  FastNoiseModelFactorN() {}

  /**
   * Constructor.
   * Example usage: FastNoiseModelFactorN(noise, key1, key2, ..., keyN)
   * @param noiseModel Shared pointer to noise model.
   * @param keys Keys for the variables in this factor, passed in as separate
   * arguments.
   */
  FastNoiseModelFactorN(const SharedNoiseModel& noiseModel,
                        KeyType<ValueTypes>... keys)
      : Base(noiseModel, std::array<Key, N>{keys...}) {}

  /**
   * Constructor.
   * Example usage: `FastNoiseModelFactorN(noise, {key1, key2, ..., keyN})`
   * Example usage: `FastNoiseModelFactorN(noise, keys)` where keys is a
   * vector<Key>
   * @param noiseModel Shared pointer to noise model.
   * @param keys A container of keys for the variables in this factor.
   */
  template <typename CONTAINER = std::initializer_list<Key>,
            typename = IsContainerOfKeys<CONTAINER>>
  FastNoiseModelFactorN(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    if (keys.size() != N) {
      throw std::invalid_argument(
          "FastNoiseModelFactorN: wrong number of keys given");
    }
  }

  /// @}

  ~FastNoiseModelFactorN() override {}

  /** Returns a key. Usage: `key<I>()` returns the I'th key.
   * I is 1-indexed for backwards compatibility/consistency!  So for example,
   * ```
   * FastNoiseModelFactorN<Pose3, Point3> factor(noise, key1, key2);
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
  virtual Vector evaluateError(
      const ValueTypes&... x,
      OptionalMatrixRefTypeT<ValueTypes>... H) const = 0;

  /// @}
  /// @name Convenience method overloads
  /// @{

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

  /* *************************************************************************
   */
  std::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    // Only linearize if the factor is active
    if (!active(x)) return std::shared_ptr<JacobianFactor>();

    const std::tuple<const ValueTypes&...> valueReferences =
        getValueReferences(std::make_index_sequence<N>{}, x);

    // Fill in terms, needed to create JacobianFactor below
    KeyVector keys_ = keys();
    std::array<size_t, N> dimensions;
    fillDimensions(valueReferences, dimensions, std::make_index_sequence<N>{});

    const auto b_dim = this->dim();

    VerticalBlockMatrix Ab_ = VerticalBlockMatrix(dimensions, b_dim, true);

    // Compute Jacobians directly into Ab_ blocks and set RHS vector b using
    // a tuple expansion over the variable pack.
    evalIntoAb(std::make_index_sequence<N>{}, valueReferences, Ab_);

    // Whiten the corresponding system now
    if (noiseModel_) {
      for (size_t j = 0; j < size(); ++j) {
        noiseModel_->whitenInPlace(Ab_(j));
      }
      Vector b = Ab_(size());
      Ab_(size()) = noiseModel_->whiten(b);
    }

    // TODO pass unwhitened + noise model to Gaussian factor
    using noiseModel::Constrained;
    if (noiseModel_ && noiseModel_->isConstrained())
      return GaussianFactor::shared_ptr(new JacobianFactor(
          std::move(keys_), std::move(Ab_),
          std::static_pointer_cast<Constrained>(noiseModel_)->unit()));
    else {
      return GaussianFactor::shared_ptr(
          new JacobianFactor(std::move(keys_), std::move(Ab_)));
    }
  }

  /// @}

 private:
  /// Helper: get the reference to the I'th value
  template <typename T, std::size_t I>
  inline const T& getValueReference(const Values& values) const {
    const Value& value = values.at(keys_[I]);

    if (dynamic_cast<const GenericValue<T>*>(&value) == nullptr) {
      throw std::invalid_argument(
          "FastNoiseModelFactorN: wrong type of value for key");
    }

    return static_cast<const GenericValue<T>&>(value).value();
  }

  /// Helper: get references to values as a tuple
  /// @param[in] sequence Index sequence for template parameter expansion
  /// @param[in] values Values object containing variable values
  /// @return Tuple of const references to the variable values
  template <std::size_t... I>
  inline std::tuple<const ValueType<I + 1>&...> getValueReferences(
      std::index_sequence<I...>, const Values& values) const {
    return std::tuple<const ValueType<I + 1>&...>(
        getValueReference<ValueType<I + 1>, I>(values)...);
  }

  /// Helper: fill dimensions array from value references tuple
  /// @param[in] valueReferences Tuple of const references to variable values
  /// @param[out] dimensions Array to fill with dimension values
  /// @param[in] sequence Index sequence for template parameter expansion
  template <std::size_t... I>
  inline void fillDimensions(
      const std::tuple<const ValueType<I + 1>&...>& valueReferences,
      std::array<size_t, N>& dimensions,
      std::index_sequence<I...>) const {
    ((dimensions[I] = traits<ValueType<I + 1>>::GetDimension(
          std::get<I>(valueReferences))),
     ...);
  }

  // Helper: expand indices to call evaluateError with values and pointers to
  // Eigen::Ref blocks that alias Ab_'s Jacobian columns. Also writes the
  // returned error into the last block (b column).
  template <std::size_t... I>
  inline void evalIntoAb(std::index_sequence<I...>,
                         const std::tuple<const ValueType<I + 1>&...>& x,
                         VerticalBlockMatrix& Ab) const {
    // Create refs that alias each Jacobian block in Ab.
    auto Htuple = std::make_tuple(Eigen::Ref<Matrix>(Ab(I))...);

    // Call evaluateError with values and pointers to the refs.
    Vector e = evaluateError(std::get<I>(x)..., (&std::get<I>(Htuple))...);

    // Store RHS vector b in the last block's single column.
    Ab(size()).col(0) = -e;  // b = -error
  }

  /** Pack expansion with index_sequence template pattern, used to index into
   * `keys_` and `H`.
   *
   * Example: For `FastNoiseModelFactorN<Pose3, Point3>`, the call would look
   * like: `const Vector error = unwhitenedError(0, 1, values, H);`
   */
  template <std::size_t... Indices>
  inline Vector unwhitenedError(gtsam::index_sequence<Indices...>,  //
                                const Values& x,
                                OptionalMatrixVecType H = nullptr) const {
    if (this->active(x)) {
      if (H) {
        // Preallocate each Jacobian matrix if not already allocated
        if (H->size() != N) H->resize(N);
        // C++17 fold expression to resize all matrices
        ((*H)[Indices].resize(
             this->dim(), traits<ValueType<Indices + 1>>::GetDimension(
                              x.at<ValueType<Indices + 1>>(keys_[Indices]))),
         ...);
        std::tuple<std::remove_pointer_t<OptionalMatrixRefTypeT<ValueTypes>>...>
            Htuple{Eigen::Ref<Matrix>((*H)[Indices])...};

        return evaluateError(x.at<ValueType<Indices + 1>>(keys_[Indices])...,
                             &std::get<Indices>(Htuple)...);
      } else {
        return evaluateError(x.at<ValueTypes>(keys_[Indices])...,
                             OptionalMatrixRefTypeT<ValueTypes>(nullptr)...);
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
        "FastNoiseModelFactor", boost::serialization::base_object<Base>(*this));
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

};  // \class FastNoiseModelFactorN

}  // namespace gtsam