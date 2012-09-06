/**
 * @file LinearContainerFactor.h
 *
 * @brief Wrap Jacobian and Hessian linear factors to allow simple injection into a nonlinear graph
 * 
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/**
 * Dummy version of a generic linear factor to be injected into a nonlinear factor graph
 */
class LinearContainerFactor : public NonlinearFactor {
protected:

	GaussianFactor::shared_ptr factor_;
	boost::optional<Values> linearizationPoint_;

	/** direct copy constructor */
	LinearContainerFactor(const GaussianFactor::shared_ptr& factor,
			const boost::optional<Values>& linearizationPoint);

public:

	/** Primary constructor: store a linear factor and decode the ordering */
	LinearContainerFactor(const JacobianFactor& factor, const Ordering& ordering,
			const Values& linearizationPoint = Values());

	/** Primary constructor: store a linear factor and decode the ordering */
	LinearContainerFactor(const HessianFactor& factor, const Ordering& ordering,
			const Values& linearizationPoint = Values());

	/** Constructor from shared_ptr */
	LinearContainerFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering,
			const Values& linearizationPoint = Values());

	/** Constructor from re-keyed factor: all indices assumed replaced with Key */
	LinearContainerFactor(const GaussianFactor::shared_ptr& factor,
			const Values& linearizationPoint = Values());

	/** Alternate constructor: store a linear factor and decode keys with inverted ordering*/
	LinearContainerFactor(const JacobianFactor& factor,
			const Ordering::InvertedMap& inverted_ordering,
			const Values& linearizationPoint = Values());

	/** Alternate constructor: store a linear factor and decode keys with inverted ordering*/
	LinearContainerFactor(const HessianFactor& factor,
			const Ordering::InvertedMap& inverted_ordering,
			const Values& linearizationPoint = Values());

	/** Constructor from shared_ptr with inverted ordering*/
	LinearContainerFactor(const GaussianFactor::shared_ptr& factor,
			const Ordering::InvertedMap& ordering,
			const Values& linearizationPoint = Values());

	// Access

	const GaussianFactor::shared_ptr& factor() const { return factor_; }

	// Testable

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const;

	// NonlinearFactor

  /**
   * Calculate the error of the factor: uses the underlying linear factor to compute ordering
   */
  double error(const Values& c) const;

  /** get the dimension of the factor: rows of linear factor */
  size_t dim() const;

  /** Extract the linearization point used in recalculating error */
	const boost::optional<Values>& linearizationPoint() const { return linearizationPoint_; }

  /** Apply the ordering to a graph - same as linearize(), but without needing a linearization point */
  GaussianFactor::shared_ptr order(const Ordering& ordering) const;

  /** linearize to a GaussianFactor: values has no effect, just clones/rekeys underlying factor */
  GaussianFactor::shared_ptr linearize(const Values& c, const Ordering& ordering) const;

  /**
   * Creates an anti-factor directly and performs rekeying due to ordering
   */
  GaussianFactor::shared_ptr negate(const Ordering& ordering) const;

  /**
   * Creates the equivalent anti-factor as another LinearContainerFactor,
   * so it remains independent of ordering.
   */
  NonlinearFactor::shared_ptr negate() const;

  /**
   * Creates a shared_ptr clone of the factor - needs to be specialized to allow
   * for subclasses
   *
   * Clones the underlying linear factor
   */
  NonlinearFactor::shared_ptr clone() const {
  	return NonlinearFactor::shared_ptr(new LinearContainerFactor(factor_,linearizationPoint_));
  }

  // casting syntactic sugar

  /**
   * Simple check whether this is a Jacobian or Hessian factor
   */
  bool isJacobian() const;

  /** Casts to JacobianFactor */
  JacobianFactor::shared_ptr toJacobian() const;

  /** Casts to HessianFactor */
  HessianFactor::shared_ptr toHessian() const;

  /**
   * Utility function for converting linear graphs to nonlinear graphs
   * consisting of LinearContainerFactors.  Two versions are available, using
   * either the ordering the linear graph was linearized around, or the inverse ordering.
   * If the inverse ordering is present, it will be faster.
   */
  static NonlinearFactorGraph convertLinearGraph(const GaussianFactorGraph& linear_graph,
  		const Ordering& ordering);

  static NonlinearFactorGraph convertLinearGraph(const GaussianFactorGraph& linear_graph,
  		const InvertedOrdering& invOrdering);

protected:
  void rekeyFactor(const Ordering& ordering);
  void rekeyFactor(const Ordering::InvertedMap& invOrdering);
  void initializeLinearizationPoint(const Values& linearizationPoint);

}; // \class LinearContainerFactor

} // \namespace gtsam


