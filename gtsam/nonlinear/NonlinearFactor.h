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
#include <cstddef>
#include <type_traits>

namespace gtsam {

/* ************************************************************************* */

/** These typedefs and aliases will help with making the evaluateError interface
 * independent of boost
 * TODO(kartikarcot): Change this to OptionalMatrixNone
 * This typedef is used to indicate that the Jacobian is not required
 * and the default value used for optional matrix pointer arguments in evaluateError.
 * Had to use the static_cast of a nullptr, because the compiler is not able to
 * deduce the type of the nullptr when expanding the evaluateError templates.
 */
#define OptionalNone static_cast<Matrix*>(nullptr)

/** This typedef will be used everywhere boost::optional<Matrix&> reference was used
 * previously. This is used to indicate that the Jacobian is optional. In the future
 * we will change this to OptionalJacobian
 */
using OptionalMatrixType = Matrix*;

/** The OptionalMatrixVecType is a pointer to a vector of matrices. It will
 * be used in situations where a vector of matrices is optional, like in 
 * unwhitenedError.
 */
using OptionalMatrixVecType = std::vector<Matrix>*;

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

  typedef std::shared_ptr<This> shared_ptr;

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
   * In nonlinear factors, the error function returns the negative log-likelihood
   * as a non-linear function of the values in a \class Values object.
   * 
   * The idea is that Gaussian factors have a quadratic error function that locally 
   * approximates the negative log-likelihood, and are obtained by \b linearizing
   * the nonlinear error function at a given linearization.
   * 
   * The derived class, \class NoiseModelFactor, adds a noise model to the factor,
   * and calculates the error by asking the user to implement the method
   * \code double evaluateError(const Values& c) const \endcode.
   */
  virtual double error(const Values& c) const;

  /**
   * The Factor::error simply extracts the \class Values from the
   * \class HybridValues and calculates the error.
   */
  double error(const HybridValues& c) const override;

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
  virtual std::shared_ptr<GaussianFactor>
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

  typedef std::shared_ptr<This> shared_ptr;

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
  virtual Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const = 0;

  /** support taking in the actual vector instead of the pointer as well
   * to get access to this version of the function from derived classes
   * one will need to use the "using" keyword and specify that like this:
   * public:
   *   using NoiseModelFactor::unwhitenedError;
   */
  Vector unwhitenedError(const Values& x, std::vector<Matrix>& H) const {
    return unwhitenedError(x, &H);
  }

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
  std::shared_ptr<GaussianFactor> linearize(const Values& x) const override;

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
namespace detail {
/** Convenience base class to add aliases `X1`, `X2`, ..., `X6` -> ValueType<N>.
 * Usage example:
 * ```
 * class MyFactor : public NoiseModelFactorN<Pose3, Point3>,
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
 * class MyFactor : public NoiseModelFactorN<Pose3, Point3> {
 *  public:
 *   using Base = NoiseModelFactorN<Pose3, Point3>;
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
class NoiseModelFactorN
    : public NoiseModelFactor,
      public detail::NoiseModelFactorAliases<ValueTypes...> {
 public:
  /// N is the number of variables (N-way factor)
  enum { N = sizeof...(ValueTypes) };

  using NoiseModelFactor::unwhitenedError;

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

  /** A helper alias to check if a list of args
   * are all references to a matrix or not. It will be used
   * to choose the right overload of evaluateError.
   */
  template <typename Ret, typename ...Args>
  using AreAllMatrixRefs = std::enable_if_t<(... && 
      std::is_convertible<Args, Matrix&>::value), Ret>;
  
  template<typename Arg>
  using IsMatrixPointer = std::is_same<typename std::decay_t<Arg>, Matrix*>;

  template<typename Arg>
  using IsNullpointer = std::is_same<typename std::decay_t<Arg>, std::nullptr_t>;

  /** A helper alias to check if a list of args
   * are all pointers to a matrix or not. It will be used
   * to choose the right overload of evaluateError.
   */
  template <typename Ret, typename ...Args>
    using AreAllMatrixPtrs = std::enable_if_t<(... &&
            (IsMatrixPointer<Args>::value || IsNullpointer<Args>::value)), Ret>;

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
   * using Factor = NoiseModelFactorN<Pose3, Point3>;
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
  Vector unwhitenedError(
      const Values& x,
      OptionalMatrixVecType H = nullptr) const override {
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
  virtual Vector evaluateError(const ValueTypes&... x,
                               OptionalMatrixTypeT<ValueTypes>... H) const = 0;

  /** If all the optional arguments are matrices then redirect the call to 
   * the one which takes pointers.
   * To get access to this version of the function from derived classes
   * one will need to use the "using" keyword and specify that like this:
   * public:
   *   using NoiseModelFactorN<list the value types here>::evaluateError;
   */
  Vector evaluateError(const ValueTypes&... x, MatrixTypeT<ValueTypes>&... H) const {
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
  template <typename... OptionalJacArgs, typename = IndexIsValid<sizeof...(OptionalJacArgs) + 1>>
  inline AreAllMatrixRefs<Vector, OptionalJacArgs...> evaluateError(const ValueTypes&... x,
                                                                  OptionalJacArgs&&... H) const {
    return evaluateError(x..., (&H)...);
  }

  /** Some (but not all) optional Jacobians are omitted (function overload)
   * and the jacobians are pointers to matrices.
   * e.g. `const Vector error = factor.evaluateError(pose, point, &Hpose);`
   */
  template <typename... OptionalJacArgs, typename = IndexIsValid<sizeof...(OptionalJacArgs) + 1>>
  inline AreAllMatrixPtrs<Vector, OptionalJacArgs...> evaluateError(const ValueTypes&... x,
                                                                    OptionalJacArgs&&... H) const {
    // If they are pointer version, ensure to cast them all to be Matrix* types
    // This will ensure any arguments inferred as std::nonetype_t are cast to (Matrix*) nullptr
    // This guides the compiler to the correct overload which is the one that takes pointers
    return evaluateError(x..., 
        std::forward<OptionalJacArgs>(H)..., static_cast<OptionalMatrixType>(OptionalNone));
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

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }

 public:
  /// @name Shortcut functions `key1()` -> `key<1>()`
  /// @{

  inline Key key1() const {
    return key<1>();
  }
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

};  // \class NoiseModelFactorN

#define NoiseModelFactor1 NoiseModelFactorN
#define NoiseModelFactor2 NoiseModelFactorN
#define NoiseModelFactor3 NoiseModelFactorN
#define NoiseModelFactor4 NoiseModelFactorN
#define NoiseModelFactor5 NoiseModelFactorN
#define NoiseModelFactor6 NoiseModelFactorN

}  // namespace gtsam
