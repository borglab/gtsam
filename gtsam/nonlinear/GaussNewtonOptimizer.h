/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussNewtonOptimizer.h
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/** Parameters for Gauss-Newton optimization, inherits from
 * NonlinearOptimizationParams.
 */
class GaussNewtonParams : public NonlinearOptimizerParams {
public:
  /** See GaussNewtonParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See GaussNewtonParams::factorization */
  enum Factorization {
    LDL,
    QR,
  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: LDL)

  GaussNewtonParams() :
    elimination(MULTIFRONTAL), factorization(LDL) {}

  ~GaussNewtonParams() {}

  virtual void print(const std::string& str = "") const {
    NonlinearOptimizerParams::print(str);
    if(elimination == MULTIFRONTAL)
      std::cout << "         elimination method: MULTIFRONTAL\n";
    else if(elimination == SEQUENTIAL)
      std::cout << "         elimination method: SEQUENTIAL\n";
    else
      std::cout << "         elimination method: (invalid)\n";

    if(factorization == LDL)
      std::cout << "       factorization method: LDL\n";
    else if(factorization == QR)
      std::cout << "       factorization method: QR\n";
    else
      std::cout << "       factorization method: (invalid)\n";

    std::cout.flush();
  }
};

/**
 * This class performs Gauss-Newton nonlinear optimization
 * TODO: use make_shared
 */
class GaussNewtonOptimizer : public NonlinearOptimizer {

public:

  typedef boost::shared_ptr<const GaussNewtonParams> SharedGNParams;
  typedef boost::shared_ptr<const Ordering> SharedOrdering;

  /// @name Standard interface
  /// @{

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const NonlinearFactorGraph& graph, const Values& values,
      const GaussNewtonParams& params = GaussNewtonParams(),
      const Ordering& ordering = Ordering()) :
        NonlinearOptimizer(
            SharedGraph(new NonlinearFactorGraph(graph)),
            SharedValues(new Values(values)),
            SharedGNParams(new GaussNewtonParams(params))),
        gnParams_(boost::static_pointer_cast<const GaussNewtonParams>(params_)),
        colamdOrdering_(ordering.size() == 0),
        ordering_(colamdOrdering_ ?
            graph_->orderingCOLAMD(*values_) : Ordering::shared_ptr(new Ordering(ordering))) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  GaussNewtonOptimizer(const SharedGraph& graph, const SharedValues& values,
      const GaussNewtonParams& params = GaussNewtonParams(),
      const SharedOrdering& ordering = SharedOrdering()) :
        NonlinearOptimizer(graph, values, SharedGNParams(new GaussNewtonParams(params))),
        gnParams_(boost::static_pointer_cast<const GaussNewtonParams>(params_)),
        colamdOrdering_(!ordering || ordering->size() == 0),
        ordering_(colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : ordering) {}

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~GaussNewtonOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual auto_ptr iterate() const;

  /** Update the graph, values, and/or parameters, leaving all other state
   * the same.  Any of these that are empty shared pointers are left unchanged
   * in the returned optimizer object.  Returns a new updated
   * NonlinearOptimzier object, the original is not modified.
   */
  virtual auto_ptr update(
      const SharedGraph& newGraph,
      const SharedValues& newValues = SharedValues(),
      const SharedParams& newParams = SharedParams()) const {
    return auto_ptr(new GaussNewtonOptimizer(*this, newGraph, newValues,
        boost::dynamic_pointer_cast<const GaussNewtonParams>(newParams)));
  }

  /** Update the ordering, leaving all other state the same.  If newOrdering
   * is an empty pointer <emph>or</emph> contains an empty Ordering object
   * (with zero size), a COLAMD ordering will be computed. Returns a new
   * NonlinearOptimizer object, the original is not modified.
   */
  virtual auto_ptr update(const SharedOrdering& newOrdering) const {
    return auto_ptr(new GaussNewtonOptimizer(*this, newOrdering)); }

  /** Create a copy of the NonlinearOptimizer */
  virtual auto_ptr clone() const {
    return auto_ptr(new GaussNewtonOptimizer(*this));
  }

  /// @}

protected:

  const SharedGNParams gnParams_;
  const bool colamdOrdering_;
  const SharedOrdering ordering_;

  /** Protected constructor called by update() to modify the graph, values, or
   * parameters.  Computes a COLAMD ordering if the optimizer was originally
   * constructed with an empty ordering, and if the graph is changing.
   */
  GaussNewtonOptimizer(const GaussNewtonOptimizer& original, const SharedGraph& newGraph,
      const SharedValues& newValues, const SharedGNParams& newParams) :
    NonlinearOptimizer(original, newGraph, newValues, newParams),
    gnParams_(newParams ? newParams : original.gnParams_),
    colamdOrdering_(original.colamdOrdering_),
    ordering_(newGraph && colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : original.ordering_) {}

  /** Protected constructor called by update() to modify the ordering, computing
   * a COLAMD ordering if the new ordering is empty, and also recomputing the
   * dimensions.
   */
  GaussNewtonOptimizer(
      const GaussNewtonOptimizer& original, const SharedOrdering& newOrdering) :
        NonlinearOptimizer(original),
        gnParams_(original.gnParams_),
        colamdOrdering_(!newOrdering || newOrdering->size() == 0),
        ordering_(colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : newOrdering) {}

private:

  // Special constructor for completing an iteration, updates the values and
  // error, and increments the iteration count.
  GaussNewtonOptimizer(const GaussNewtonOptimizer& original,
      const SharedValues& newValues, double newError) :
        NonlinearOptimizer(graph_, newValues, params_, newError, iterations_+1),
        gnParams_(original.gnParams_),
        colamdOrdering_(original.colamdOrdering_),
        ordering_(original.ordering_) {}
};

}
