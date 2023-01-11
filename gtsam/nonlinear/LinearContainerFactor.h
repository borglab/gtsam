/**
 * @file LinearContainerFactor.h
 *
 * @brief Wrap Jacobian and Hessian linear factors to allow simple injection into a nonlinear graph
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/std_optional_serialization.h>

#include <optional>

namespace gtsam {

  // Forward declarations
  class JacobianFactor;
  class HessianFactor;

/**
 * Dummy version of a generic linear factor to be injected into a nonlinear factor graph
 *
 * This factor does have the ability to perform relinearization under small-angle and
 * linearity assumptions if a linearization point is added.
 */
class GTSAM_EXPORT LinearContainerFactor : public NonlinearFactor {
protected:

  GaussianFactor::shared_ptr factor_;
  std::optional<Values> linearizationPoint_;

  /** direct copy constructor */
  LinearContainerFactor(const GaussianFactor::shared_ptr& factor, const std::optional<Values>& linearizationPoint);

  // Some handy typedefs
  typedef NonlinearFactor Base;
  typedef LinearContainerFactor This;

public:

  typedef boost::shared_ptr<This> shared_ptr;

  /** Default constructor - necessary for serialization */
  LinearContainerFactor() {}

  /** Primary constructor: store a linear factor with optional linearization point */
  LinearContainerFactor(const JacobianFactor& factor, const Values& linearizationPoint = Values());

  /** Primary constructor: store a linear factor with optional linearization point */
  LinearContainerFactor(const HessianFactor& factor, const Values& linearizationPoint = Values());

  /** Constructor from shared_ptr */
  LinearContainerFactor(const GaussianFactor::shared_ptr& factor, const Values& linearizationPoint = Values());

  // Access

  const GaussianFactor::shared_ptr& factor() const { return factor_; }

  // Testable

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override;

  // NonlinearFactor

  /**
   * Calculate the nonlinear error for the factor, where the error is computed
   * by passing the delta between linearization point and c, where
   * delta = linearizationPoint_.localCoordinates(c), into the error function
   * of the stored linear factor.
   *
   * @return nonlinear error if linearizationPoint present, zero otherwise
   */
  double error(const Values& c) const override;

  /** get the dimension of the factor: rows of linear factor */
  size_t dim() const override;

  /** Extract the linearization point used in recalculating error */
  const std::optional<Values>& linearizationPoint() const { return linearizationPoint_; }

  /**
   * Linearize to a GaussianFactor, with method depending on the presence of a linearizationPoint
   *  - With no linearization point, returns a cloned version of the stored linear factor.
   *  - With a linearization point provided, returns a relinearized version of
   *  the linearized factor.
   *
   * The relinearization approach used computes a linear delta between the original linearization
   * point and the new values c, where delta = linearizationPoint_.localCoordinates(c), and
   * substitutes this change into the system.  This substitution is only really valid for
   * linear variable manifolds, and for any variables based on a non-commutative
   * manifold (such as Pose2, Pose3), the relinearized version will be effective
   * for only small angles.
   *
   * TODO: better approximation of relinearization
   * TODO: switchable modes for approximation technique
   */
  GaussianFactor::shared_ptr linearize(const Values& c) const override;

  /**
   * Creates an anti-factor directly
   */
  GaussianFactor::shared_ptr negateToGaussian() const;

  /**
   * Creates the equivalent anti-factor as another LinearContainerFactor.
   */
  NonlinearFactor::shared_ptr negateToNonlinear() const;

  /**
   * Creates a shared_ptr clone of the factor - needs to be specialized to allow
   * for subclasses
   *
   * Clones the underlying linear factor
   */
  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(new LinearContainerFactor(factor_,linearizationPoint_));
  }

  /**
   * Creates a shared_ptr clone of the
   * factor with different keys using
   * a map from old->new keys
   */
  NonlinearFactor::shared_ptr rekey(
      const std::map<Key, Key>& rekey_mapping) const override;

  /**
   * Clones a factor and fully replaces its keys
   * @param new_keys is the full replacement set of keys
   */
  NonlinearFactor::shared_ptr rekey(const KeyVector& new_keys) const override;

  /// Casting syntactic sugar
  inline bool hasLinearizationPoint() const { return linearizationPoint_.has_value(); }

  /**
   * Simple checks whether this is a Jacobian or Hessian factor
   */
  bool isJacobian() const;
  bool isHessian() const;

  /** Casts to JacobianFactor */
  boost::shared_ptr<JacobianFactor> toJacobian() const;

  /** Casts to HessianFactor */
  boost::shared_ptr<HessianFactor> toHessian() const;

  /**
   * Utility function for converting linear graphs to nonlinear graphs
   * consisting of LinearContainerFactors.
   */
  static NonlinearFactorGraph ConvertLinearGraph(const GaussianFactorGraph& linear_graph,
      const Values& linearizationPoint = Values());

 protected:
  void initializeLinearizationPoint(const Values& linearizationPoint);

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NonlinearFactor",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(factor_);
    ar & BOOST_SERIALIZATION_NVP(linearizationPoint_);
  }

}; // \class LinearContainerFactor

template<> struct traits<LinearContainerFactor> : public Testable<LinearContainerFactor> {};

} // \namespace gtsam

