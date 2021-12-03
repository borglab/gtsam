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

/**
 * A convenient base class for creating your own NoiseModelFactor with n
 * variables.  To derive from this class, implement evaluateError().
 *
 * Templated on a values structure type. The values structures are typically
 * more general than just vectors, e.g., Rot3 or Pose3,
 * which are objects in non-linear manifolds (Lie groups).
 */
template <class... VALUES>
class NoiseModelFactorN : public NoiseModelFactor {
 public:
  /** The type of the N'th template param can be obtained with VALUE<N> */
  template <int N>
  using VALUE = typename std::tuple_element<N, std::tuple<VALUES...>>::type;

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
  /**
   * Default Constructor for I/O
   */
  NoiseModelFactorN() {}

  /**
   * Constructor.
   * Example usage: NoiseModelFactorN(noise, key1, key2, ..., keyN)
   * @param noiseModel shared pointer to noise model
   * @param keys... keys for the variables in this factor
   */
  NoiseModelFactorN(const SharedNoiseModel& noiseModel,
                    key_type<VALUES>... keys)
      : Base(noiseModel, std::array<Key, sizeof...(VALUES)>{keys...}) {}

  /**
   * Constructor.
   * @param noiseModel shared pointer to noise model
   * @param keys a container of keys for the variables in this factor
   */
  template <typename CONTAINER>
  NoiseModelFactorN(const SharedNoiseModel& noiseModel, CONTAINER keys)
      : Base(noiseModel, keys) {
    assert(keys.size() == sizeof...(VALUES));
  }

  ~NoiseModelFactorN() override {}

  /** Method to retrieve keys */
  template <int N>
  inline Key key() const {
    return keys_[N];
  }

  /** Calls the n-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  Vector unwhitenedError(
      const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const override {
    return unwhitenedError(boost::mp11::index_sequence_for<VALUES...>{}, x, H);
  }

  /**
   *  Override this method to finish implementing an n-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should
   * compute both the function evaluation and its derivative(s) in the requested
   * variables.
   */
  virtual Vector evaluateError(const VALUES&... x,
                               optional_matrix_type<VALUES>... H) const = 0;

  /** No-jacobians requested function overload (since parameter packs can't have
   * default args).  This specializes the version below to avoid recursive calls
   * since this is commonly used. */
  inline Vector evaluateError(const VALUES&... x) const {
    return evaluateError(x..., optional_matrix_type<VALUES>()...);
  }

  /** Some optional jacobians omitted function overload */
  template <typename... OptionalJacArgs,
            typename std::enable_if<(sizeof...(OptionalJacArgs) > 0) &&
                                        (sizeof...(OptionalJacArgs) <
                                         sizeof...(VALUES)),
                                    bool>::type = true>
  inline Vector evaluateError(const VALUES&... x,
                              OptionalJacArgs&&... H) const {
    return evaluateError(x..., std::forward<OptionalJacArgs>(H)...,
                         boost::none);
  }

 private:
  /** Pack expansion with index_sequence template pattern */
  template <std::size_t... Inds>
  Vector unwhitenedError(
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

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 1
 * variable.  To derive from this class, implement evaluateError(). */
template <class VALUE>
class NoiseModelFactor1 : public NoiseModelFactorN<VALUE> {
 public:
  // aliases for value types pulled from keys
  using X = VALUE;

 protected:
  using Base = NoiseModelFactor;  // grandparent, for backwards compatibility
  using This = NoiseModelFactor1<VALUE>;

 public:
  // inherit NoiseModelFactorN's constructors
  using NoiseModelFactorN<VALUE>::NoiseModelFactorN;
  ~NoiseModelFactor1() override {}

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
/** A convenient base class for creating your own NoiseModelFactor with 2
 * variables.  To derive from this class, implement evaluateError(). */
template <class VALUE1, class VALUE2>
class NoiseModelFactor2 : public NoiseModelFactorN<VALUE1, VALUE2> {
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
/** A convenient base class for creating your own NoiseModelFactor with 3
 * variables.  To derive from this class, implement evaluateError(). */
template <class VALUE1, class VALUE2, class VALUE3>
class NoiseModelFactor3 : public NoiseModelFactorN<VALUE1, VALUE2, VALUE3> {
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
/** A convenient base class for creating your own NoiseModelFactor with 4
 * variables.  To derive from this class, implement evaluateError(). */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4>
class NoiseModelFactor4
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
/** A convenient base class for creating your own NoiseModelFactor with 5
 * variables.  To derive from this class, implement evaluateError(). */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5>
class NoiseModelFactor5
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
/** A convenient base class for creating your own NoiseModelFactor with 6
 * variables.  To derive from this class, implement evaluateError(). */
template <class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
          class VALUE6>
class NoiseModelFactor6
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
