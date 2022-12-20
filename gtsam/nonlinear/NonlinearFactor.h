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
template <class... ValueTypes>
class NoiseModelFactorN : public NoiseModelFactor {
 public:
  /// N is the number of variables (N-way factor)
  enum { N = sizeof...(ValueTypes) };

  /// The type of the i'th template param can be obtained as ValueType<I>
  template <int I, typename std::enable_if<(I < N), bool>::type = true>
  using ValueType =
      typename std::tuple_element<I, std::tuple<ValueTypes...>>::type;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactorN<ValueTypes...>;

  /* Like std::void_t, except produces `boost::optional<Matrix&>` instead. Used
   * to expand fixed-type parameter-packs with same length as ValueTypes */
  template <typename T>
  using OptionalMatrix = boost::optional<Matrix&>;

  /* Like std::void_t, except produces `Key` instead. Used to expand fixed-type
   * parameter-packs with same length as ValueTypes */
  template <typename T>
  using KeyType = Key;

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
   * Example usage: NoiseModelFactorN(noise, {key1, key2, ..., keyN})
   * Example usage: NoiseModelFactorN(noise, keys); where keys is a vector<Key>
   * @param noiseModel Shared pointer to noise model.
   * @param keys A container of keys for the variables in this factor.
   */
  template <typename CONTAINER = std::initializer_list<Key>,
            // check that CONTAINER is a container of Keys:
            typename T = typename std::decay<
                decltype(*std::declval<CONTAINER>().begin())>::type,
            typename std::enable_if<std::is_convertible<T, Key>::value,
                                    bool>::type = true>
  NoiseModelFactorN(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    assert(keys.size() == N);
  }

  /// @}

  ~NoiseModelFactorN() override {}

  /// Returns a key. Usage: `key<I>()` returns the I'th key.
  template <int I>
  inline typename std::enable_if<(I < N), Key>::type key() const {
    return keys_[I];
  }

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
    return unwhitenedError(boost::mp11::index_sequence_for<ValueTypes...>{}, x,
                           H);
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
  virtual Vector evaluateError(const ValueTypes&... x,
                               OptionalMatrix<ValueTypes>... H) const = 0;

  /// @}
  /// @name Convenience method overloads
  /// @{

  /** No-jacobians requested function overload (since parameter packs can't have
   * default args).  This specializes the version below to avoid recursive calls
   * since this is commonly used.
   *
   * e.g. `Vector error = factor.evaluateError(x1, x2, x3);`
   */
  inline Vector evaluateError(const ValueTypes&... x) const {
    return evaluateError(x..., OptionalMatrix<ValueTypes>()...);
  }

  /** Some optional jacobians omitted function overload */
  template <typename... OptionalJacArgs,
            typename std::enable_if<(sizeof...(OptionalJacArgs) > 0) &&
                                        (sizeof...(OptionalJacArgs) < N),
                                    bool>::type = true>
  inline Vector evaluateError(const ValueTypes&... x,
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
        return evaluateError(x.at<ValueTypes>(keys_[Inds])..., (*H)[Inds]...);
      } else {
        return evaluateError(x.at<ValueTypes>(keys_[Inds])...);
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

/* ************************************************************************* */
/** @deprecated: use NoiseModelFactorN, replacing .key() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 1 variable.  To derive from this class, implement evaluateError().
 */
template <class VALUE>
class GTSAM_DEPRECATED NoiseModelFactor1 : public NoiseModelFactorN<VALUE> {
 public:
  // aliases for value types pulled from keys, for backwards compatibility
  using X = VALUE;

 protected:
  using Base = NoiseModelFactor;  // grandparent, for backwards compatibility
  using This = NoiseModelFactor1<VALUE>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE>::NoiseModelFactorN;
  ~NoiseModelFactor1() override {}

  /** method to retrieve key */
  inline Key key() const { return this->keys_[0]; }

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
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 2 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2>
class GTSAM_DEPRECATED NoiseModelFactor2
    : public NoiseModelFactorN<VALUE1, VALUE2> {
 public:
  // aliases for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor2<VALUE1, VALUE2>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2>::NoiseModelFactorN;
  ~NoiseModelFactor2() override {}

  /** methods to retrieve both keys */
  inline Key key1() const { return this->keys_[0]; }
  inline Key key2() const { return this->keys_[1]; }

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
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 3 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3>
class GTSAM_DEPRECATED NoiseModelFactor3
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3> {
 public:
  // aliases for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;
  using X3 = VALUE3;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor3<VALUE1, VALUE2, VALUE3>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3>::NoiseModelFactorN;
  ~NoiseModelFactor3() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return this->keys_[0]; }
  inline Key key2() const { return this->keys_[1]; }
  inline Key key3() const { return this->keys_[2]; }

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
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 4 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4>
class GTSAM_DEPRECATED NoiseModelFactor4
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4> {
 public:
  // aliases for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;
  using X3 = VALUE3;
  using X4 = VALUE4;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor4<VALUE1, VALUE2, VALUE3, VALUE4>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4>::NoiseModelFactorN;
  ~NoiseModelFactor4() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return this->keys_[0]; }
  inline Key key2() const { return this->keys_[1]; }
  inline Key key3() const { return this->keys_[2]; }
  inline Key key4() const { return this->keys_[3]; }

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
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 5 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
class GTSAM_DEPRECATED NoiseModelFactor5
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5> {
 public:
  // aliases for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;
  using X3 = VALUE3;
  using X4 = VALUE4;
  using X5 = VALUE5;

 protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor5<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4,
                          VALUE5>::NoiseModelFactorN;
  ~NoiseModelFactor5() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return this->keys_[0]; }
  inline Key key2() const { return this->keys_[1]; }
  inline Key key3() const { return this->keys_[2]; }
  inline Key key4() const { return this->keys_[3]; }
  inline Key key5() const { return this->keys_[4]; }

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
/** @deprecated: use NoiseModelFactorN, replacing .key1() with .key<0> and X1
 * with ValueType<0>.
 * A convenient base class for creating your own NoiseModelFactor
 * with 6 variables.  To derive from this class, implement evaluateError().
 */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
          class VALUE6>
class GTSAM_DEPRECATED NoiseModelFactor6
    : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6> {
 public:
  // aliases for value types pulled from keys
  using X1 = VALUE1;
  using X2 = VALUE2;
  using X3 = VALUE3;
  using X4 = VALUE4;
  using X5 = VALUE5;
  using X6 = VALUE6;

 protected:
  using Base = NoiseModelFactor;
  using This =
      NoiseModelFactor6<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5,
                          VALUE6>::NoiseModelFactorN;
  ~NoiseModelFactor6() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return this->keys_[0]; }
  inline Key key2() const { return this->keys_[1]; }
  inline Key key3() const { return this->keys_[2]; }
  inline Key key4() const { return this->keys_[3]; }
  inline Key key5() const { return this->keys_[4]; }
  inline Key key6() const { return this->keys_[5]; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};  // \class NoiseModelFactor6

} // \namespace gtsam
