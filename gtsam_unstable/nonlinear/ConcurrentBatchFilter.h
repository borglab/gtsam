/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchFilter.h
 * @brief   A Levenberg-Marquardt Batch Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <queue>

namespace gtsam {

/**
 * A Levenberg-Marquardt Batch Filter that implements the Concurrent Filtering and Smoother interface.
 */
class GTSAM_UNSTABLE_EXPORT ConcurrentBatchFilter : public ConcurrentFilter {

public:
  typedef boost::shared_ptr<ConcurrentBatchFilter> shared_ptr;
  typedef ConcurrentFilter Base; ///< typedef for base class

  /** Meta information returned about the update */
  struct Result {
    size_t iterations; ///< The number of optimizer iterations performed
    size_t lambdas; ///< The number of different L-M lambda factors that were tried during optimization
    size_t nonlinearVariables; ///< The number of variables that can be relinearized
    size_t linearVariables; ///< The number of variables that must keep a constant linearization point
    double error; ///< The final factor graph error

    /// Constructor
    Result() : iterations(0), lambdas(0), nonlinearVariables(0), linearVariables(0), error(0) {};

    /// Getter methods
    size_t getIterations() const { return iterations; }
    size_t getLambdas() const { return lambdas; }
    size_t getNonlinearVariables() const { return nonlinearVariables; }
    size_t getLinearVariables() const { return linearVariables; }
    double getError() const { return error; }
  };

  /** Default constructor */
  ConcurrentBatchFilter(const LevenbergMarquardtParams& parameters = LevenbergMarquardtParams(), bool relin = true) : parameters_(parameters), relin_(relin) {};

  /** Default destructor */
  virtual ~ConcurrentBatchFilter() {};

  /** Implement a GTSAM standard 'print' function */
  void print(const std::string& s = "Concurrent Batch Filter:\n", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /** Check if two Concurrent Filters are equal */
  bool equals(const ConcurrentFilter& rhs, double tol = 1e-9) const;

  /** Access the current set of factors */
  const NonlinearFactorGraph& getFactors() const {
    return factors_;
  }

  /** Access the current linearization point */
  const Values& getLinearizationPoint() const {
    return theta_;
  }

  /** Access the current ordering */
  const OrderingOrdered& getOrdering() const {
    return ordering_;
  }

  /** Access the current set of deltas to the linearization point */
  const VectorValuesOrdered& getDelta() const {
    return delta_;
  }

  /** Compute the current best estimate of all variables and return a full Values structure.
   * If only a single variable is needed, it may be faster to call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const {
    return theta_.retract(delta_, ordering_);
  }

  /** Compute the current best estimate of a single variable. This is generally faster than
   * calling the no-argument version of calculateEstimate if only specific variables are needed.
   * @param key
   * @return
   */
  template<class VALUE>
  VALUE calculateEstimate(Key key) const {
    const Index index = ordering_.at(key);
    const Vector delta = delta_.at(index);
    return theta_.at<VALUE>(key).retract(delta);
  }

  /**
   * Add new factors and variables to the filter.
   *
   * Add new measurements, and optionally new variables, to the filter.
   * This runs a full update step of the derived filter algorithm
   *
   * @param newFactors The new factors to be added to the smoother
   * @param newTheta Initialization points for new variables to be added to the filter
   * You must include here all new variables occurring in newFactors that were not already
   * in the filter.
   * @param keysToMove An optional set of keys to remove from the filter and
   */
  Result update(const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(), const Values& newTheta = Values(),
      const boost::optional<FastList<Key> >& keysToMove = boost::none);

protected:

  LevenbergMarquardtParams parameters_;  ///< LM parameters
  bool relin_;
  NonlinearFactorGraph factors_;  ///< The set of all factors currently in the filter
  Values theta_;  ///< Current linearization point of all variables in the filter
  OrderingOrdered ordering_; ///< The current ordering used to calculate the linear deltas
  VectorValuesOrdered delta_; ///< The current set of linear deltas from the linearization point
  std::queue<size_t> availableSlots_; ///< The set of available factor graph slots caused by deleting factors
  Values separatorValues_; ///< The linearization points of the separator variables. These should not be updated during optimization.
  std::vector<size_t> smootherSummarizationSlots_;  ///< The slots in factor graph that correspond to the current smoother summarization factors

  // Storage for information to be sent to the smoother
  NonlinearFactorGraph filterSummarization_; ///< A temporary holding place for calculated filter summarization factors to be sent to the smoother
  NonlinearFactorGraph smootherFactors_;  ///< A temporary holding place for the set of full nonlinear factors being sent to the smoother
  Values smootherValues_; ///< A temporary holding place for the linearization points of all keys being sent to the smoother

  /**
   * Perform any required operations before the synchronization process starts.
   * Called by 'synchronize'
   */
  virtual void presync();

  /**
   * Populate the provided containers with factors that constitute the filter branch summarization
   * needed by the smoother. Also, linearization points for the new root clique must be provided.
   *
   * @param summarizedFactors The summarized factors for the filter branch
   * @param rootValues The linearization points of the root clique variables
   */
  virtual void getSummarizedFactors(NonlinearFactorGraph& summarizedFactors, Values& separatorValues);

  /**
   * Populate the provided containers with factors being sent to the smoother from the filter. These
   * may be original nonlinear factors, or factors encoding a summarization of the filter information.
   * The specifics will be implementation-specific for a given filter.
   *
   * @param smootherFactors The new factors to be added to the smoother
   * @param smootherValues The linearization points of any new variables
   */
  virtual void getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues);

  /**
   * Apply the updated version of the smoother branch summarized factors.
   *
   * @param summarizedFactors An updated version of the smoother branch summarized factors
   */
  virtual void synchronize(const NonlinearFactorGraph& summarizedFactors, const Values& separatorValues);

  /**
   * Perform any required operations after the synchronization process finishes.
   * Called by 'synchronize'
   */
  virtual void postsync();


private:

  /** Augment the graph with new factors
   *
   * @param factors The factors to add to the graph
   * @return The slots in the graph where they were inserted
   */
  std::vector<size_t> insertFactors(const NonlinearFactorGraph& factors);

  /** Remove factors from the graph by slot index
   *
   * @param slots The slots in the factor graph that should be deleted
   */
  void removeFactors(const std::vector<size_t>& slots);

  /** Use colamd to update into an efficient ordering */
  void reorder(const boost::optional<FastList<Key> >& keysToMove = boost::none);

  /** Use a modified version of L-M to update the linearization point and delta */
  static Result optimize(const NonlinearFactorGraph& factors, Values& theta, const OrderingOrdered& ordering,
       VectorValuesOrdered& delta, const Values& linearValues, const LevenbergMarquardtParams& parameters);

  /** Marginalize out the set of requested variables from the filter, caching them for the smoother
   *  This effectively moves the separator.
   */
  void marginalize(const FastList<Key>& keysToMove);

  /** Marginalize out the set of requested variables from the filter, caching them for the smoother
   *  This effectively moves the separator.
   */
  static NonlinearFactorGraph marginalize(const NonlinearFactorGraph& graph, const Values& values,
      const OrderingOrdered& ordering, const std::set<Key>& marginalizeKeys, const GaussianFactorGraphOrdered::Eliminate& function = EliminateQROrdered);

  /** Print just the nonlinear keys in a nonlinear factor */
  static void PrintNonlinearFactor(const NonlinearFactor::shared_ptr& factor,
      const std::string& indent = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter);

  /** Print just the nonlinear keys in a linear factor */
  static void PrintLinearFactor(const GaussianFactorOrdered::shared_ptr& factor, const OrderingOrdered& ordering,
      const std::string& indent = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter);

  // A custom elimination tree that supports forests and partial elimination
  class EliminationForest {
  public:
    typedef boost::shared_ptr<EliminationForest> shared_ptr; ///< Shared pointer to this class

  private:
    typedef FastList<GaussianFactorOrdered::shared_ptr> Factors;
    typedef FastList<shared_ptr> SubTrees;
    typedef std::vector<GaussianConditionalOrdered::shared_ptr> Conditionals;

    Index key_; ///< index associated with root
    Factors factors_; ///< factors associated with root
    SubTrees subTrees_; ///< sub-trees

    /** default constructor, private, as you should use Create below */
    EliminationForest(Index key = 0) : key_(key) {}

    /**
     * Static internal function to build a vector of parent pointers using the
     * algorithm of Gilbert et al., 2001, BIT.
     */
    static std::vector<Index> ComputeParents(const VariableIndexOrdered& structure);

    /** add a factor, for Create use only */
    void add(const GaussianFactorOrdered::shared_ptr& factor) { factors_.push_back(factor); }

    /** add a subtree, for Create use only */
    void add(const shared_ptr& child) { subTrees_.push_back(child); }

  public:

    /** return the key associated with this tree node */
    Index key() const { return key_; }

    /** return the const reference of children */
    const SubTrees& children() const { return subTrees_; }

    /** return the const reference to the factors */
    const Factors& factors() const { return factors_; }

    /** Create an elimination tree from a factor graph */
    static std::vector<shared_ptr> Create(const GaussianFactorGraphOrdered& factorGraph, const VariableIndexOrdered& structure);

    /** Recursive routine that eliminates the factors arranged in an elimination tree */
    GaussianFactorOrdered::shared_ptr eliminateRecursive(GaussianFactorGraphOrdered::Eliminate function);

    /** Recursive function that helps find the top of each tree */
    static void removeChildrenIndices(std::set<Index>& indices, const EliminationForest::shared_ptr& tree);
  };

}; // ConcurrentBatchFilter

/// Typedef for Matlab wrapping
typedef ConcurrentBatchFilter::Result ConcurrentBatchFilterResult;

}/// namespace gtsam
