/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.h
 * @brief   Non-linear factor base classes
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Gerry Chen
 */

// \callgraph

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/utilities.h>  // boost::index_sequence

#include <boost/serialization/base_object.hpp>
#include <boost/assign/list_of.hpp>

namespace gtsam {

using boost::assign::cref_list_of;

/* ************************************************************************* */

/**
 * Nonlinear factor base class
 *
 * \nosubgrouping
 */
class GTSAM_EXPORT NonlinearFactor: public Factor {

protected:

  // Some handy typedefs
  typedef Factor Base;
  typedef NonlinearFactor This;

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /** Default constructor for I/O only */
  NonlinearFactor() {}

  /**
   * Constructor from a collection of the keys involved in this factor
   */
  template<typename CONTAINER>
  NonlinearFactor(const CONTAINER& keys) :
    Base(keys) {}

  /// @}
  /// @name Testable
  /// @{

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const;
  /// @}
  /// @name Standard Interface
  /// @{

  /** Destructor */
  virtual ~NonlinearFactor() {}


  /**
   * Calculate the error of the factor
   * This is typically equal to log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/sigma^2 \f$ in case of Gaussian.
   * You can override this for systems with unusual noise models.
   */
  virtual double error(const Values& c) const = 0;

  /** get the dimension of the factor (number of rows on linearization) */
  virtual size_t dim() const = 0;

  /**
   * Checks whether a factor should be used based on a set of values.
   * This is primarily used to implement inequality constraints that
   * require a variable active set. For all others, the default implementation
   * returning true solves this problem.
   *
   * In an inequality/bounding constraint, this active() returns true
   * when the constraint is *NOT* fulfilled.
   * @return true if the constraint is active
   */
  virtual bool active(const Values& /*c*/) const { return true; }

  /** linearize to a GaussianFactor */
  virtual boost::shared_ptr<GaussianFactor>
  linearize(const Values& c) const = 0;

  /**
   * Creates a shared_ptr clone of the factor - needs to be specialized to allow
   * for subclasses
   *
   * By default, throws exception if subclass does not implement the function.
   */
  virtual shared_ptr clone() const {
    // TODO: choose better exception to throw here
    throw std::runtime_error("NonlinearFactor::clone(): Attempting to clone factor with no clone() implemented!");
    return shared_ptr();
  }

  /**
   * Creates a shared_ptr clone of the
   * factor with different keys using
   * a map from old->new keys
   */
  virtual shared_ptr rekey(const std::map<Key,Key>& rekey_mapping) const;

  /**
   * Clones a factor and fully replaces its keys
   * @param new_keys is the full replacement set of keys
   */
  virtual shared_ptr rekey(const KeyVector& new_keys) const;

  /**
   * Should the factor be evaluated in the same thread as the caller
   * This is to enable factors that has shared states (like the Python GIL lock)
   */
   virtual bool sendable() const {
    return true;
  }

}; // \class NonlinearFactor

/// traits
template<> struct traits<NonlinearFactor> : public Testable<NonlinearFactor> {
};

/* ************************************************************************* */
/**
 * A nonlinear sum-of-squares factor with a zero-mean noise model
 * implementing the density \f$ P(z|x) \propto exp -0.5*|z-h(x)|^2_C \f$
 * Templated on the parameter type X and the values structure Values
 * There is no return type specified for h(x). Instead, we require
 * the derived class implements \f$ \mathtt{error\_vector}(x) = h(x)-z \approx A \delta x - b \f$
 * This allows a graph to have factors with measurements of mixed type.

 * The noise model is typically Gaussian, but robust and constrained error models are also supported.
 */
class GTSAM_EXPORT NoiseModelFactor: public NonlinearFactor {

protected:

  // handy typedefs
  typedef NonlinearFactor Base;
  typedef NoiseModelFactor This;

  SharedNoiseModel noiseModel_; /** Noise model */

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor for I/O only */
  NoiseModelFactor() {}

  /** Destructor */
  ~NoiseModelFactor() override {}

  /**
   * Constructor
   */
  template<typename CONTAINER>
  NoiseModelFactor(const SharedNoiseModel& noiseModel, const CONTAINER& keys) :
    Base(keys), noiseModel_(noiseModel) {}

protected:

  /**
   * Constructor - only for subclasses, as this does not set keys.
   */
  NoiseModelFactor(const SharedNoiseModel& noiseModel) : noiseModel_(noiseModel) {}

public:

  /** Print */
  void print(const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override;

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const override {
    return noiseModel_->dim();
  }

  /// access to the noise model
  const SharedNoiseModel& noiseModel() const {
    return noiseModel_;
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * Override this method to finish implementing an N-way factor.
   * If the optional arguments is specified, it should compute
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const = 0;

  /**
   * Vector of errors, whitened
   * This is the raw error, i.e., i.e. \f$ (h(x)-z)/\sigma \f$ in case of a Gaussian
   */
  Vector whitenedError(const Values& c) const;

  /**
   * Vector of errors, whitened, but unweighted by any loss function
   */
  Vector unweightedWhitenedError(const Values& c) const;

  /**
   * Compute the effective weight of the factor from the noise model.
   */
  double weight(const Values& c) const;

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  double error(const Values& c) const override;

  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const override;

  /**
   * Creates a shared_ptr clone of the
   * factor with a new noise model
   */
  shared_ptr cloneWithNewNoiseModel(const SharedNoiseModel newNoise) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor",
         boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
  }

}; // \class NoiseModelFactor


/* ************************************************************************* */
/**
 * A convenient base class for creating your own NoiseModelFactor
 * with n variables.  To derive from this class, implement evaluateError().
 *
 * For example, a 2-way factor that computes the difference in x-translation
 * between a Pose3 and Point3 could be implemented like so:
 *
 * ~~~~~~~~~~~~~~~~~~~~{.cpp}
 * class MyFactor : public NoiseModelFactorN<Pose3, Point3> {
 *  public:
 *   using Base = NoiseModelFactorN<Pose3, Point3>;
 *
 *   MyFactor(Key pose_key, Key point_key, const SharedNoiseModel& noiseModel)
 *       : Base(noiseModel, pose_key, point_key) {}
 *
 *   Vector evaluateError(
 *       const Pose3& T, const Point3& p,
 *       boost::optional<Matrix&> H_T = boost::none,
 *       boost::optional<Matrix&> H_p = boost::none) const override {
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
class NoiseModelFactorN : public NoiseModelFactor {
 public:
  /// N is the number of variables (N-way factor)
  enum { N = sizeof...(ValueTypes) };

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactorN<ValueTypes...>;

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

  /// @}

  /* Like std::void_t, except produces `boost::optional<Matrix&>` instead of
   * `void`. Used to expand fixed-type parameter-packs with same length as
   * ValueTypes. */
  template <typename T>
  using OptionalMatrix = boost::optional<Matrix&>;

  /* Like std::void_t, except produces `Key` instead of `void`. Used to expand
   * fixed-type parameter-packs with same length as ValueTypes. */
  template <typename T>
  using KeyType = Key;

 public:
  /**
   * The type of the I'th template param can be obtained as ValueType<I>.
   * I is 1-indexed for backwards compatibility/consistency!  So for example,
   * ```
   * using Factor = NoiseModelFactorN<Pose3, Point3>;
   * Factor::ValueType<1>  // Pose3
   * Factor::ValueType<2>  // Point3
   * // Factor::ValueType<0> // ERROR!  Will not compile.
   * // Factor::ValueType<3> // ERROR!  Will not compile.
   * ```
   */
  template <int I, typename = IndexIsValid<I>>
  using ValueType =
      typename std::tuple_element<I - 1, std::tuple<ValueTypes...>>::type;

 public:

  /// @name Constructors
  /// @{

  /// Default Constructor for I/O
  NoiseModelFactorN() {}

  /**
   * Constructor.
   * Example usage: NoiseModelFactorN(noise, key1, key2, ..., keyN)
   * @param noiseModel Shared pointer to noise model.
   * @param keys Keys for the variables in this factor, passed in as separate
   * arguments.
   */
  NoiseModelFactorN(const SharedNoiseModel& noiseModel,
                    KeyType<ValueTypes>... keys)
      : Base(noiseModel, std::array<Key, N>{keys...}) {}

  /**
   * Constructor.
   * Example usage: `NoiseModelFactorN(noise, {key1, key2, ..., keyN})`
   * Example usage: `NoiseModelFactorN(noise, keys)` where keys is a vector<Key>
   * @param noiseModel Shared pointer to noise model.
   * @param keys A container of keys for the variables in this factor.
   */
  template <typename CONTAINER = std::initializer_list<Key>,
            typename = IsContainerOfKeys<CONTAINER>>
  NoiseModelFactorN(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    if (keys.size() != N) {
      throw std::invalid_argument(
          "NoiseModelFactorN: wrong number of keys given");
    }
  }

  /// @}

  ~NoiseModelFactorN() override {}

  /** Returns a key. Usage: `key<I>()` returns the I'th key.
   * I is 1-indexed for backwards compatibility/consistency!  So for example,
   * ```
   * NoiseModelFactorN<Pose3, Point3> factor(noise, key1, key2);
   * key<1>()  // = key1
   * key<2>()  // = key2
   * // key<0>()  // ERROR!  Will not compile
   * // key<3>()  // ERROR!  Will not compile
   * ```
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
  Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    return unwhitenedError(boost::mp11::index_sequence_for<ValueTypes...>{}, x,
                           H);
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
   *     boost::optional<Matrix&> H1 = boost::none,
   *     boost::optional<Matrix&> H2 = boost::none) const override { ... }
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
  virtual Vector evaluateError(const ValueTypes&... x,
                               OptionalMatrix<ValueTypes>... H) const = 0;

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
    return evaluateError(x..., OptionalMatrix<ValueTypes>()...);
  }

  /** Some (but not all) optional Jacobians are omitted (function overload)
   *
   * e.g. `const Vector error = factor.evaluateError(pose, point, Hpose);`
   */
  template <typename... OptionalJacArgs,
            typename = IndexIsValid<sizeof...(OptionalJacArgs) + 1>>
  inline Vector evaluateError(const ValueTypes&... x,
                              OptionalJacArgs&&... H) const {
    return evaluateError(x..., std::forward<OptionalJacArgs>(H)...,
                         boost::none);
  }

  /// @}

 private:
  /** Pack expansion with index_sequence template pattern, used to index into
   * `keys_` and `H`.
   *
   * Example: For `NoiseModelFactorN<Pose3, Point3>`, the call would look like:
   *    `const Vector error = unwhitenedError(0, 1, values, H);`
   */
  template <std::size_t... Indices>
  inline Vector unwhitenedError(
      boost::mp11::index_sequence<Indices...>,  //
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
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

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }

 public:
  /// @name Deprecated methods.  Use `key<1>()`, `key<2>()`, ... instead of old
  /// `key1()`, `key2()`, ...
  /// If your class is templated AND you are trying to call `key<1>` inside your
  /// class, due to dependent types you need to do `this->template key<1>()`.
  /// @{

  inline Key GTSAM_DEPRECATED key1() const {
    return key<1>();
  }
  template <int I = 2>
  inline Key GTSAM_DEPRECATED key2() const {
    static_assert(I <= N, "Index out of bounds");
    return key<2>();
  }
  template <int I = 3>
  inline Key GTSAM_DEPRECATED key3() const {
    static_assert(I <= N, "Index out of bounds");
    return key<3>();
  }
  template <int I = 4>
  inline Key GTSAM_DEPRECATED key4() const {
    static_assert(I <= N, "Index out of bounds");
    return key<4>();
  }
  template <int I = 5>
  inline Key GTSAM_DEPRECATED key5() const {
    static_assert(I <= N, "Index out of bounds");
    return key<5>();
  }
  template <int I = 6>
  inline Key GTSAM_DEPRECATED key6() const {
    static_assert(I <= N, "Index out of bounds");
    return key<6>();
  }

  /// @}

};  // \class NoiseModelFactorN

/******************************************************************************
 * THE REMAINDER OF THIS FILE IS JUST FOR DEPRECATED BACKWARD COMPATIBILITY   *
 * DEFINITIONS.  DO NOT USE THESE FOR NEW CODE                                *
 ******************************************************************************/

/** Convenience macros to add deprecated typedefs `X1`, `X2`, ..., `X6`.
 * This was only used to maintain backwards compatibility of existing factors!
 * Do NOT use for new factors!
 * When transitioning from NoiseModelFactor1 to NoiseModelFactorN, this macro
 * was used to add deprecated typedefs for the old NoiseModelFactor1.
 * Usage example:
 * ```
 * class MyFactor : public NoiseModelFactorN<Pose3, Point3> {
 *  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(MyFactor, 2);
 *  // class implementation ...
 * };
 * 
 * // MyFactor::X1 == Pose3
 * // MyFactor::X2 == Point3
 * ```
 */
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS1(CLASS) \
  using X GTSAM_DEPRECATED = typename CLASS::template ValueType<1>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS1_(CLASS) \
  using X1 GTSAM_DEPRECATED = typename CLASS::template ValueType<1>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS2(CLASS) \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS1_(CLASS)      \
  using X2 GTSAM_DEPRECATED = typename CLASS::template ValueType<2>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS3(CLASS) \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS2(CLASS)       \
  using X3 GTSAM_DEPRECATED = typename CLASS::template ValueType<3>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS4(CLASS) \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS3(CLASS)       \
  using X4 GTSAM_DEPRECATED = typename CLASS::template ValueType<4>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS5(CLASS) \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS4(CLASS)       \
  using X5 GTSAM_DEPRECATED = typename CLASS::template ValueType<5>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS6(CLASS) \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS5(CLASS)       \
  using X6 GTSAM_DEPRECATED = typename CLASS::template ValueType<6>;
#define ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(CLASS, N) \
 public:                                                       \
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS##N(CLASS);      \
 private:

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 1 variable.  To derive from this class, implement evaluateError().
 */
CLANG_DIAGNOSTIC_PUSH_IGNORE("-Wdeprecated-declarations")  // Silence warnings
GCC_DIAGNOSTIC_PUSH_IGNORE("-Wdeprecated-declarations")    // while we define
MSVC_DIAGNOSTIC_PUSH_IGNORE(4996)                          // deprecated classes
template <class VALUE>
class GTSAM_DEPRECATED NoiseModelFactor1 : public NoiseModelFactorN<VALUE> {
 public:
  /** Aliases for value types pulled from keys, for backwards compatibility.
   * Note: in your code you can probably just do:
   *  `using X = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X = typename NoiseModelFactor1::template ValueType<1>;

 protected:
  using Base = NoiseModelFactor;  // grandparent, for backwards compatibility
  using This = NoiseModelFactor1<VALUE>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE>::NoiseModelFactorN;
  ~NoiseModelFactor1() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor1

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 2 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2>
class GTSAM_DEPRECATED NoiseModelFactor2
    : public NoiseModelFactorN<VALUE1, VALUE2> {
 public:
  /** Aliases for value types pulled from keys.
   * Note: in your code you can probably just do: 
   *  `using X1 = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X1 = typename NoiseModelFactor2::template ValueType<1>;
  using X2 = typename NoiseModelFactor2::template ValueType<2>;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor2<VALUE1, VALUE2>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2>::NoiseModelFactorN;
  ~NoiseModelFactor2() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor2

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 3 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3>
class GTSAM_DEPRECATED NoiseModelFactor3
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3> {
 public:
  /** Aliases for value types pulled from keys.
   * Note: in your code you can probably just do: 
   *  `using X1 = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X1 = typename NoiseModelFactor3::template ValueType<1>;
  using X2 = typename NoiseModelFactor3::template ValueType<2>;
  using X3 = typename NoiseModelFactor3::template ValueType<3>;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor3<VALUE1, VALUE2, VALUE3>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3>::NoiseModelFactorN;
  ~NoiseModelFactor3() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor3

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 4 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4>
class GTSAM_DEPRECATED NoiseModelFactor4
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4> {
 public:
  /** Aliases for value types pulled from keys.
   * Note: in your code you can probably just do: 
   *  `using X1 = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X1 = typename NoiseModelFactor4::template ValueType<1>;
  using X2 = typename NoiseModelFactor4::template ValueType<2>;
  using X3 = typename NoiseModelFactor4::template ValueType<3>;
  using X4 = typename NoiseModelFactor4::template ValueType<4>;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor4<VALUE1, VALUE2, VALUE3, VALUE4>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4>::NoiseModelFactorN;
  ~NoiseModelFactor4() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor4

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 5 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
class GTSAM_DEPRECATED NoiseModelFactor5
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5> {
 public:
  /** Aliases for value types pulled from keys.
   * Note: in your code you can probably just do: 
   *  `using X1 = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X1 = typename NoiseModelFactor5::template ValueType<1>;
  using X2 = typename NoiseModelFactor5::template ValueType<2>;
  using X3 = typename NoiseModelFactor5::template ValueType<3>;
  using X4 = typename NoiseModelFactor5::template ValueType<4>;
  using X5 = typename NoiseModelFactor5::template ValueType<5>;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor5<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4,
                          VALUE5>::NoiseModelFactorN;
  ~NoiseModelFactor5() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor5

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<1>() and X1
 * with ValueType<1>.
 * If your class is templated AND you are trying to call `.key<1>()` or
 * `ValueType<1>` inside your class, due to dependent types you need to do
 * `this->template key<1>()` or `This::template ValueType<1>`.
 * ~~~
 * A convenient base class for creating your own NoiseModelFactor
 * with 6 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
          class VALUE6>
class GTSAM_DEPRECATED NoiseModelFactor6
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6> {
 public:
  /** Aliases for value types pulled from keys.
   * Note: in your code you can probably just do: 
   *  `using X1 = ValueType<1>;`
   * but this class is uglier due to dependent types.
   * See e.g. testNonlinearFactor.cpp:TestFactorN.
   */
  using X1 = typename NoiseModelFactor6::template ValueType<1>;
  using X2 = typename NoiseModelFactor6::template ValueType<2>;
  using X3 = typename NoiseModelFactor6::template ValueType<3>;
  using X4 = typename NoiseModelFactor6::template ValueType<4>;
  using X5 = typename NoiseModelFactor6::template ValueType<5>;
  using X6 = typename NoiseModelFactor6::template ValueType<6>;

 protected:
  using Base = NoiseModelFactor;
  using This =
      NoiseModelFactor6<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5,
                          VALUE6>::NoiseModelFactorN;
  ~NoiseModelFactor6() override {}

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor6
DIAGNOSTIC_POP() // Finish silencing warnings

} // \namespace gtsam
