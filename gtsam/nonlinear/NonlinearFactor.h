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
 */

// \callgraph

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/serialization/base_object.hpp>
#include <boost/assign/list_of.hpp>

#if BOOST_VERSION >= 106600
#include <boost/mp11/integer_sequence.hpp>
#else
namespace boost {
namespace mp11 {
// Adapted from https://stackoverflow.com/a/32223343/9151520
template <size_t... Ints>
struct index_sequence {
  using type = index_sequence;
  using value_type = size_t;
  static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};
namespace detail {
template <class Sequence1, class Sequence2>
struct _merge_and_renumber;

template <size_t... I1, size_t... I2>
struct _merge_and_renumber<index_sequence<I1...>, index_sequence<I2...> >
    : index_sequence<I1..., (sizeof...(I1) + I2)...> {};
}  // namespace detail
template <size_t N>
struct make_index_sequence
    : detail::_merge_and_renumber<
          typename make_index_sequence<N / 2>::type,
          typename make_index_sequence<N - N / 2>::type> {};
template <>
struct make_index_sequence<0> : index_sequence<> {};
template <>
struct make_index_sequence<1> : index_sequence<0> {};
template <class... T>
using index_sequence_for = make_index_sequence<sizeof...(T)>;
}  // namespace mp11
}  // namespace boost
#endif

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
/* We need some helper structs to help us with NoiseModelFactorN - specifically
 * we need to alias X1, X2, X3, ... in the templated NoiseModelFactorN class to
 * maintain backwards compatibility with NoiseModelFactor1, NoiseModelFactor2,
 * NoiseModelFactor3, ...
 *
 * The tricky part is that we want to _conditionally_ alias these only if the
 * `sizeof...(VALUES)` is greater than the index we want to alias (e.g. a 3-way
 * factor should only have up to X3).  SFINAE doesn't work in this case with
 * aliases so we have to come up with a different approach.
 *
 * The approach we use is to inherit from structs that conditionally typedef
 * these types for us (using template specialization).  Note: std::conditional
 * doesn't work because it requires that both types exist at compile time.
 *
 * Usage:
 * ```
 * template <class... VALUES>
 * class MyClass : public AliasX3<VALUES...> { ... };
 * ```
 * This will only typedef X3 if VALUES has at least 3 template parameters.  So
 * then we can do something like:
 * ```
 * int main {
 *   MyClass<bool, int, double>::X3 a;  // variable a will have type double
 *   // MyClass<bool, int>::X3 b;   // this won't compile
 *   MyClass<bool, int, char, double>::X3 c;  // variable c will have type char
 * }
 * ```
 */

namespace detail {

// By default, we do not alias X (empty struct).
#define ALIAS_FALSE_X(NAME)        \
  template <bool, class... VALUES> \
  struct Alias##NAME##_ {};
// But if the first template is true, then we do alias X by specializing.
#define ALIAS_TRUE_X(NAME, N)                                                 \
  template <class... VALUES>                                                  \
  struct Alias##NAME##_<true, VALUES...> {                                    \
    using NAME = typename std::tuple_element<N, std::tuple<VALUES...>>::type; \
  };
// Finally, alias a convenience struct that chooses the right version.
#define ALIAS_X(NAME, N, CONDITION)     \
  ALIAS_FALSE_X(NAME)        \
  ALIAS_TRUE_X(NAME, N)      \
  template <class... VALUES> \
  using Alias##NAME = Alias##NAME##_<(CONDITION), VALUES...>;

ALIAS_X(X, 0, 0 == sizeof...(VALUES));
ALIAS_X(X1, 0, 0 < sizeof...(VALUES));
ALIAS_X(X2, 1, 1 < sizeof...(VALUES));
ALIAS_X(X3, 2, 2 < sizeof...(VALUES));
ALIAS_X(X4, 3, 3 < sizeof...(VALUES));
ALIAS_X(X5, 4, 4 < sizeof...(VALUES));
ALIAS_X(X6, 5, 5 < sizeof...(VALUES));
#undef ALIAS_FALSE_X
#undef ALIAS_TRUE_X
#undef ALIAS_X

}  // namespace detail

/* ************************************************************************* */
/**
 * A convenient base class for creating your own NoiseModelFactor
 * with n variables.  To derive from this class, implement evaluateError().
 *
 * For example, a 2-way factor could be implemented like so:
 *
 * ~~~~~~~~~~~~~~~~~~~~{.cpp}
 * class MyFactor : public NoiseModelFactorN<double, double> {
 *  public:
 *   using Base = NoiseModelFactorN<double, double>;
 *
 *   MyFactor(Key key1, Key key2, const SharedNoiseModel& noiseModel)
 *       : Base(noiseModel, key1, key2) {}
 *
 *   Vector evaluateError(
 *       const double& x1, const double& x2,
 *       boost::optional<Matrix&> H1 = boost::none,
 *       boost::optional<Matrix&> H2 = boost::none) const override {
 *     if (H1) *H1 = (Matrix(1, 1) << 1.0).finished();
 *     if (H2) *H2 = (Matrix(1, 1) << 2.0).finished();
 *     return (Vector(1) << x1 + 2 * x2).finished();
 *   }
 * };
 * ~~~~~~~~~~~~~~~~~~~~
 *
 * These factors are templated on a values structure type. The values structures
 * are typically more general than just vectors, e.g., Rot3 or Pose3, which are
 * objects in non-linear manifolds (Lie groups).
 */
template <class... VALUES>
class NoiseModelFactorN
    : public NoiseModelFactor,
      public detail::AliasX<VALUES...>,   // using X = VALUE1
      public detail::AliasX1<VALUES...>,  // using X1 = VALUE1
      public detail::AliasX2<VALUES...>,  // using X2 = VALUE2
      public detail::AliasX3<VALUES...>,  // using X3 = VALUE3
      public detail::AliasX4<VALUES...>,  // using X4 = VALUE4
      public detail::AliasX5<VALUES...>,  // using X5 = VALUE5
      public detail::AliasX6<VALUES...>   // using X6 = VALUE6
{
 public:
  /// N is the number of variables (N-way factor)
  enum { N = sizeof...(VALUES) };

  /** The type of the i'th template param can be obtained as VALUE<I> */
  template <int I, typename std::enable_if<(I < N), bool>::type = true>
  using VALUE = typename std::tuple_element<I, std::tuple<VALUES...>>::type;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactorN<VALUES...>;

  /* "Dummy templated" alias is used to expand fixed-type parameter packs with
   * same length as VALUES.  This ignores the template parameter. */
  template <typename T>
  using optional_matrix_type = boost::optional<Matrix&>;

  /* "Dummy templated" alias is used to expand fixed-type parameter packs with
   * same length as VALUES.  This ignores the template parameter. */
  template <typename T>
  using key_type = Key;

 public:
  /// @name Constructors
  /// @{

  /**
   * Default Constructor for I/O
   */
  NoiseModelFactorN() {}

  /**
   * Constructor.
   * Example usage: NoiseModelFactorN(noise, key1, key2, ..., keyN)
   * @param noiseModel Shared pointer to noise model.
   * @param keys Keys for the variables in this factor, passed in as separate
   * arguments.
   */
  NoiseModelFactorN(const SharedNoiseModel& noiseModel,
                    key_type<VALUES>... keys)
      : Base(noiseModel, std::array<Key, N>{keys...}) {}

  /**
   * Constructor.  Only enabled for n-ary factors where n > 1.
   * @param noiseModel Shared pointer to noise model.
   * @param keys A container of keys for the variables in this factor.
   */
  template <typename CONTAINER,  // use "dummy" parameter T to delay deduction
            size_t T = N, typename std::enable_if<(T > 1), bool>::type = true>
  NoiseModelFactorN(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    assert(keys.size() == N);
  }

  /// @}

  ~NoiseModelFactorN() override {}

  /** Methods to retrieve keys */
#define SUB(Old, New) template <int Old = New>  // to delay template deduction
#define KEY_IF_TRUE(Enable) typename std::enable_if<(Enable), Key>::type
  // templated version of `key<I>()`
  template <int I>
  inline KEY_IF_TRUE(I < N) key() const {
    return keys_[I];
  }
  // backwards-compatibility functions
  SUB(T, N) inline KEY_IF_TRUE(T == 1) key() const { return keys_[0]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 1) key1() const { return keys_[0]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 2) key2() const { return keys_[1]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 3) key3() const { return keys_[2]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 4) key4() const { return keys_[3]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 5) key5() const { return keys_[4]; }
  SUB(T, N) inline KEY_IF_TRUE(T >= 6) key6() const { return keys_[5]; }
#undef SUB
#undef KEY_IF_TRUE

  /// @name NoiseModelFactor methods
  /// @{

  /** Calls the n-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class.
   * @param[in] x A Values object containing the values of all the variables
   * used in this factor
   * @param[out] H A vector of (dynamic) matrices whose size should be equal to
   * n.  The jacobians w.r.t. each variable will be output in this parameter.
   */
  Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    return unwhitenedError(boost::mp11::index_sequence_for<VALUES...>{}, x, H);
  }

  /// @}
  /// @name Virtual methods
  /// @{
  /**
   *  Override this method to finish implementing an n-way factor.
   *
   *  Both the `x` and `H` arguments are written here as parameter packs, but
   * when overriding this method, you probably want to explicitly write them
   * out.  For example, for a 2-way factor with variable types Pose3 and double:
   * ```
   * Vector evaluateError(const Pose3& x1, const double& x2,
   *                      boost::optional<Matrix&> H1 = boost::none,
   *                      boost::optional<Matrix&> H2 = boost::none) const
   * override {...}
   * ```
   *
   *  If any of the optional Matrix reference arguments are specified, it should
   * compute both the function evaluation and its derivative(s) in the requested
   * variables.
   *
   * @param x The values of the variables to evaluate the error for.  Passed in
   * as separate arguments.
   * @param[out] H The Jacobian with respect to each variable (optional).
   */
  virtual Vector evaluateError(const VALUES&... x,
                               optional_matrix_type<VALUES>... H) const = 0;

  /// @}
  /// @name Convenience method overloads
  /// @{

  /** No-jacobians requested function overload (since parameter packs can't have
   * default args).  This specializes the version below to avoid recursive calls
   * since this is commonly used.
   *
   * e.g. `Vector error = factor.evaluateError(x1, x2, x3);`
   */
  inline Vector evaluateError(const VALUES&... x) const {
    return evaluateError(x..., optional_matrix_type<VALUES>()...);
  }

  /** Some optional jacobians omitted function overload */
  template <typename... OptionalJacArgs,
            typename std::enable_if<(sizeof...(OptionalJacArgs) > 0) &&
                                        (sizeof...(OptionalJacArgs) < N),
                                    bool>::type = true>
  inline Vector evaluateError(const VALUES&... x,
                              OptionalJacArgs&&... H) const {
    return evaluateError(x..., std::forward<OptionalJacArgs>(H)...,
                         boost::none);
  }

  /// @}

 private:
  /** Pack expansion with index_sequence template pattern, used to index into
   * `keys_` and `H`
   */
  template <std::size_t... Inds>
  inline Vector unwhitenedError(
      boost::mp11::index_sequence<Inds...>,  //
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (this->active(x)) {
      if (H) {
        return evaluateError(x.at<VALUES>(keys_[Inds])..., (*H)[Inds]...);
      } else {
        return evaluateError(x.at<VALUES>(keys_[Inds])...);
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
};  // \class NoiseModelFactorN

// // `using` does not work for some reason
// template <class VALUE>
// using NoiseModelFactor1 = NoiseModelFactorN<VALUE>;
// template <class VALUE1, class VALUE2>
// using NoiseModelFactor2 = NoiseModelFactorN<VALUE1, VALUE2>;
// template <class VALUE1, class VALUE2, class VALUE3>
// using NoiseModelFactor3 = NoiseModelFactorN<VALUE1, VALUE2, VALUE3>;
// template <class VALUE1, class VALUE2, class VALUE3, class VALUE4>
// using NoiseModelFactor4 = NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4>;
// template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
// using NoiseModelFactor5 =
//     NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5>;
// template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
//           class VALUE6>
// using NoiseModelFactor6 =
//     NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6>;

// this is visually ugly
template <class VALUE>
struct NoiseModelFactor1 : NoiseModelFactorN<VALUE> {
  using NoiseModelFactorN<VALUE>::NoiseModelFactorN;
  using This = NoiseModelFactor1<VALUE>;
};
template <class VALUE1, class VALUE2>
struct NoiseModelFactor2 : NoiseModelFactorN<VALUE1, VALUE2> {
  using NoiseModelFactorN<VALUE1, VALUE2>::NoiseModelFactorN;
  using This = NoiseModelFactor2<VALUE1, VALUE2>;
};
template <class VALUE1, class VALUE2, class VALUE3>
struct NoiseModelFactor3 : NoiseModelFactorN<VALUE1, VALUE2, VALUE3> {
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3>::NoiseModelFactorN;
  using This = NoiseModelFactor3<VALUE1, VALUE2, VALUE3>;
};
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4>
struct NoiseModelFactor4 : NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4> {
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4>::NoiseModelFactorN;
  using This = NoiseModelFactor4<VALUE1, VALUE2, VALUE3, VALUE4>;
};
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
struct NoiseModelFactor5
    : NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5> {
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4,
                          VALUE5>::NoiseModelFactorN;
  using This = NoiseModelFactor5<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5>;
};
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
          class VALUE6>
struct NoiseModelFactor6
    : NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6> {
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5,
                          VALUE6>::NoiseModelFactorN;
  using This =
      NoiseModelFactor6<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6>;
};

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN
 * Convenient base classes for creating your own NoiseModelFactors with 1-6
 * variables.  To derive from these classes, implement evaluateError().
 */
// // This has the side-effect that you could e.g. NoiseModelFactor6<double>
// #define NoiseModelFactor1 NoiseModelFactorN
// #define NoiseModelFactor2 NoiseModelFactorN
// #define NoiseModelFactor3 NoiseModelFactorN
// #define NoiseModelFactor4 NoiseModelFactorN
// #define NoiseModelFactor5 NoiseModelFactorN
// #define NoiseModelFactor6 NoiseModelFactorN

} // \namespace gtsam
