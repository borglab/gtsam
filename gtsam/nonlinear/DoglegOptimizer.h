/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DoglegOptimizer.h
 * @brief   
 * @author  Richard Roberts
 * @created Feb 26, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/** Parameters for Levenberg-Marquardt optimization.  Note that this parameters
 * class inherits from NonlinearOptimizerParams, which specifies the parameters
 * common to all nonlinear optimization algorithms.  This class also contains
 * all of those parameters.
 */
class DoglegParams : public NonlinearOptimizerParams {
public:
  /** See DoglegParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See DoglegParams::factorization */
  enum Factorization {
    LDL,
    QR,
  };

  /** See DoglegParams::dlVerbosity */
  enum DLVerbosity {
    SILENT,
    VERBOSE
  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: LDL)
  double deltaInitial; ///< The initial trust region radius (default: 1.0)
  DLVerbosity dlVerbosity; ///< The verbosity level for Dogleg (default: SILENT), see also NonlinearOptimizerParams::verbosity

  DoglegParams() :
    elimination(MULTIFRONTAL), factorization(LDL), deltaInitial(1.0), dlVerbosity(SILENT) {}

  virtual ~DoglegParams() {}

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

    std::cout << "               deltaInitial: " << deltaInitial << "\n";

    std::cout.flush();
  }
};

/**
 * This class performs Dogleg nonlinear optimization
 * TODO: use make_shared
 */
class DoglegOptimizer : public NonlinearOptimizer {

public:

  typedef boost::shared_ptr<const DoglegParams> SharedDLParams;
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
  DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& values,
      const DoglegParams& params = DoglegParams(),
      const Ordering& ordering = Ordering()) :
        NonlinearOptimizer(
            SharedGraph(new NonlinearFactorGraph(graph)),
            SharedValues(new Values(values)),
            SharedDLParams(new DoglegParams(params))),
        dlParams_(boost::static_pointer_cast<const DoglegParams>(params_)),
        colamdOrdering_(ordering.size() == 0),
        ordering_(colamdOrdering_ ?
            graph_->orderingCOLAMD(*values_) : Ordering::shared_ptr(new Ordering(ordering))),
        dimensions_(new vector<size_t>(values_->dims(*ordering_))),
        delta_(dlParams_->deltaInitial) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.  For convenience this
   * version takes plain objects instead of shared pointers, but internally
   * copies the objects.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const NonlinearFactorGraph& graph, const Values& values,
      const Ordering& ordering) :
        NonlinearOptimizer(
            SharedGraph(new NonlinearFactorGraph(graph)),
            SharedValues(new Values(values)),
            SharedDLParams(new DoglegParams())),
        dlParams_(boost::static_pointer_cast<const DoglegParams>(params_)),
        colamdOrdering_(ordering.size() == 0),
        ordering_(colamdOrdering_ ?
            graph_->orderingCOLAMD(*values_) : Ordering::shared_ptr(new Ordering(ordering))),
        dimensions_(new vector<size_t>(values_->dims(*ordering_))),
        delta_(dlParams_->deltaInitial) {}

  /** Standard constructor, requires a nonlinear factor graph, initial
   * variable assignments, and optimization parameters.
   * @param graph The nonlinear factor graph to optimize
   * @param values The initial variable assignments
   * @param params The optimization parameters
   */
  DoglegOptimizer(const SharedGraph& graph, const SharedValues& values,
      const DoglegParams& params = DoglegParams(),
      const SharedOrdering& ordering = SharedOrdering()) :
        NonlinearOptimizer(graph, values, SharedDLParams(new DoglegParams(params))),
        dlParams_(boost::static_pointer_cast<const DoglegParams>(params_)),
        colamdOrdering_(!ordering || ordering->size() == 0),
        ordering_(colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : ordering),
        dimensions_(new vector<size_t>(values_->dims(*ordering_))),
        delta_(dlParams_->deltaInitial) {}

  /** Access the variable ordering used by this optimizer */
  const SharedOrdering& ordering() const { return ordering_; }

  /// @}

  /// @name Advanced interface
  /// @{

  /** Virtual destructor */
  virtual ~DoglegOptimizer() {}

  /** Perform a single iteration, returning a new NonlinearOptimizer class
   * containing the updated variable assignments, which may be retrieved with
   * values().
   */
  virtual auto_ptr iterate() const;

  /** Update the graph, values, and/or parameters, leaving all other state
   * the same.  Any of these that are empty shared pointers are left unchanged
   * in the returned optimizer object.  Returns a new updated NonlinearOptimizer
   * object, the original is not modified.
   */
  virtual auto_ptr update(
      const SharedGraph& newGraph,
      const SharedValues& newValues = SharedValues(),
      const SharedParams& newParams = SharedParams()) const {
    return auto_ptr(new DoglegOptimizer(*this, newGraph, newValues,
        boost::dynamic_pointer_cast<const DoglegParams>(newParams)));
  }

  /** Update the ordering, leaving all other state the same.  If newOrdering
   * is an empty pointer <emph>or</emph> contains an empty Ordering object
   * (with zero size), a COLAMD ordering will be computed. Returns a new
   * NonlinearOptimizer object, the original is not modified.
   */
  virtual auto_ptr update(const SharedOrdering& newOrdering) const {
    return auto_ptr(new DoglegOptimizer(*this, newOrdering)); }

  /** Update the damping factor lambda, leaving all other state the same.
   * Returns a new NonlinearOptimizer object, the original is not modified.
   */
  virtual auto_ptr update(double newDelta) const {
    return auto_ptr(new DoglegOptimizer(*this, newDelta)); }

  /** Create a copy of the NonlinearOptimizer */
  virtual auto_ptr clone() const {
    return auto_ptr(new DoglegOptimizer(*this));
  }

  /// @}

protected:

  typedef boost::shared_ptr<const std::vector<size_t> > SharedDimensions;

  const SharedDLParams dlParams_;
  const bool colamdOrdering_;
  const SharedOrdering ordering_;
  const SharedDimensions dimensions_;
  const double delta_;

  /** Protected constructor called by update() to modify the graph, values, or
   * parameters.  Computes a COLAMD ordering if the optimizer was originally
   * constructed with an empty ordering, and if the graph is changing.
   * Recomputes the dimensions if the values are changing.
   */
  DoglegOptimizer(const DoglegOptimizer& original, const SharedGraph& newGraph,
      const SharedValues& newValues, const SharedDLParams& newParams) :
        NonlinearOptimizer(original, newGraph, newValues, newParams),
        dlParams_(newParams ? newParams : original.dlParams_),
        colamdOrdering_(original.colamdOrdering_),
        ordering_(newGraph && colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : original.ordering_),
        dimensions_(newValues ?
            SharedDimensions(new std::vector<size_t>(values_->dims(*ordering_))) : original.dimensions_),
        delta_(original.delta_) {}

  /** Protected constructor called by update() to modify the ordering, computing
   * a COLAMD ordering if the new ordering is empty, and also recomputing the
   * dimensions.
   */
  DoglegOptimizer(
      const DoglegOptimizer& original, const SharedOrdering& newOrdering) :
        NonlinearOptimizer(original),
        dlParams_(original.dlParams_),
        colamdOrdering_(!newOrdering || newOrdering->size() == 0),
        ordering_(colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : newOrdering),
        dimensions_(new std::vector<size_t>(values_->dims(*ordering_))),
        delta_(original.delta_) {}

  /** Protected constructor called by update() to modify lambda. */
  DoglegOptimizer(
      const DoglegOptimizer& original, double newDelta) :
        NonlinearOptimizer(original),
        dlParams_(original.dlParams_),
        colamdOrdering_(original.colamdOrdering_),
        ordering_(original.ordering_),
        dimensions_(original.dimensions_),
        delta_(newDelta) {}

private:

  // Special constructor for completing an iteration, updates the values and
  // error, and increments the iteration count.
  DoglegOptimizer(const DoglegOptimizer& original,
      const SharedValues& newValues, double newError, double newDelta) :
        NonlinearOptimizer(original.graph_, newValues, original.params_, newError, original.iterations_+1),
        dlParams_(original.dlParams_),
        colamdOrdering_(original.colamdOrdering_),
        ordering_(original.ordering_),
        dimensions_(original.dimensions_),
        delta_(newDelta) {}
};

}
