/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <vector>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
#include <stdexcept>

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastList.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam {

//typedef std::vector<GaussianFactor::shared_ptr> CachedFactors;

template<class CONDITIONAL, class VALUES>
class ISAM2: public BayesTree<CONDITIONAL> {

protected:

  // current linearization point
  VALUES theta_;

  // VariableIndex lets us look up factors by involved variable and keeps track of dimensions
  VariableIndex variableIndex_;

  // the linear solution, an update to the estimate in theta
  VectorValues deltaUnpermuted_;

  // The residual permutation through which the deltaUnpermuted_ is
  // referenced.  Permuting the VectorVALUES is slow, so for performance the
  // permutation is applied at access time instead of to the VectorVALUES
  // itself.
  Permuted<VectorValues> delta_;

  // for keeping all original nonlinear factors
  NonlinearFactorGraph<VALUES> nonlinearFactors_;

  // The "ordering" allows converting Symbols to Index (integer) keys.  We
  // keep it up to date as we add and reorder variables.
  Ordering ordering_;

  // cached intermediate results for restarting computation in the middle
  //	CachedFactors cached_;

#ifndef NDEBUG
  std::vector<bool> lastRelinVariables_;
#endif

public:

  typedef BayesTree<CONDITIONAL> Base;
  typedef ISAM2<CONDITIONAL, VALUES> This;

  /** Create an empty Bayes Tree */
  ISAM2();

  //	/** Create a Bayes Tree from a Bayes Net */
  //	ISAM2(const NonlinearFactorGraph<VALUES>& fg, const Ordering& ordering, const VALUES& config);

  /** Destructor */
  virtual ~ISAM2() {}

  typedef typename BayesTree<CONDITIONAL>::sharedClique sharedClique;
  typedef typename BayesTree<CONDITIONAL>::Cliques Cliques;
  typedef JacobianFactor CacheFactor;

  /**
   * ISAM2.
   */
  void update(const NonlinearFactorGraph<VALUES>& newFactors, const VALUES& newTheta,
      double wildfire_threshold = 0., double relinearize_threshold = 0., bool relinearize = true,
      bool force_relinearize = false);

  // needed to create initial estimates
  const VALUES& getLinearizationPoint() const {return theta_;}

  // estimate based on incomplete delta (threshold!)
  VALUES calculateEstimate() const;

  // estimate based on full delta (note that this is based on the current linearization point)
  VALUES calculateBestEstimate() const;

  const Permuted<VectorValues>& getDelta() const { return delta_; }

  const NonlinearFactorGraph<VALUES>& getFactorsUnsafe() const { return nonlinearFactors_; }

  const Ordering& getOrdering() const { return ordering_; }

  size_t lastAffectedVariableCount;
  size_t lastAffectedFactorCount;
  size_t lastAffectedCliqueCount;
  size_t lastAffectedMarkedCount;
  size_t lastBacksubVariableCount;
  size_t lastNnzTop;

  boost::shared_ptr<FastSet<Index> > replacedKeys;

private:

  FastList<size_t> getAffectedFactors(const FastList<Index>& keys) const;
  FactorGraph<GaussianFactor>::shared_ptr relinearizeAffectedFactors(const FastList<Index>& affectedKeys) const;
  FactorGraph<CacheFactor> getCachedBoundaryFactors(Cliques& orphans);

  boost::shared_ptr<FastSet<Index> > recalculate(const FastSet<Index>& markedKeys, const FastSet<Index>& structuralKeys, const std::vector<Index>& newKeys, const FactorGraph<GaussianFactor>::shared_ptr newFactors = FactorGraph<GaussianFactor>::shared_ptr());
  //	void linear_update(const GaussianFactorGraph& newFactors);
  void find_all(sharedClique clique, FastSet<Index>& keys, const std::vector<bool>& marked); // helper function

public:

  struct Impl {
    static void AddVariables(const VALUES& newTheta, VALUES& theta, Permuted<VectorValues>& delta, Ordering& ordering, typename Base::Nodes& nodes);
  };

}; // ISAM2

} /// namespace gtsam
