/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalFixedLagSmoother.cpp
 * @brief   An iSAM2-based fixed-lag smoother. To the extent possible, this class mimics the iSAM2
 * interface. However, additional parameters, such as the smoother lag and the timestamp associated
 * with each variable are needed.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/base/debug.h>

#include <gtsam/linear/GaussianSequentialSolver.h>

namespace gtsam {

/* ************************************************************************* */
void IncrementalFixedLagSmoother::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool IncrementalFixedLagSmoother::equals(const FixedLagSmoother& rhs, double tol) const {
  const IncrementalFixedLagSmoother* e =  dynamic_cast<const IncrementalFixedLagSmoother*> (&rhs);
  return e != NULL
      && FixedLagSmoother::equals(*e, tol)
      && isam_.equals(e->isam_, tol);
}

/* ************************************************************************* */
bool checkMarginalizeLeaves(ISAM2& isam, const FastList<Key>& leafKeys) {

  Matrix expectedAugmentedHessian, expected3AugmentedHessian;

  vector<Index> toKeep;

  const Index lastVar = isam.getOrdering().size() - 1;

  for (Index i = 0; i <= lastVar; ++i)

    if (find(leafKeys.begin(), leafKeys.end(), isam.getOrdering().key(i))
        == leafKeys.end())

      toKeep.push_back(i);

  // Calculate expected marginal from iSAM2 tree

  GaussianFactorGraph isamAsGraph(isam);

  GaussianSequentialSolver solver(isamAsGraph);

  GaussianFactorGraph marginalgfg = *solver.jointFactorGraph(toKeep);

  expectedAugmentedHessian = marginalgfg.augmentedHessian();

  //// Calculate expected marginal from cached linear factors

  //assert(isam.params().cacheLinearizedFactors);

  //GaussianSequentialSolver solver2(isam.linearFactors_, isam.params().factorization == ISAM2Params::QR);

  //expected2AugmentedHessian = solver2.jointFactorGraph(toKeep)->augmentedHessian();

  // Calculate expected marginal from original nonlinear factors

  GaussianSequentialSolver solver3(

      *isam.getFactorsUnsafe().linearize(isam.getLinearizationPoint(),
          isam.getOrdering()),

      isam.params().factorization == ISAM2Params::QR);

  expected3AugmentedHessian =
      solver3.jointFactorGraph(toKeep)->augmentedHessian();

  // Do marginalization

  isam.experimentalMarginalizeLeaves(leafKeys);

  // Check

  GaussianFactorGraph actualMarginalGraph(isam);

  Matrix actualAugmentedHessian = actualMarginalGraph.augmentedHessian();

  //Matrix actual2AugmentedHessian = linearFactors_.augmentedHessian();

  Matrix actual3AugmentedHessian = isam.getFactorsUnsafe().linearize(

  isam.getLinearizationPoint(), isam.getOrdering())->augmentedHessian();

  assert(
      actualAugmentedHessian.unaryExpr(std::ptr_fun(&std::isfinite<double>)).all());

  // Check full marginalization

  bool treeEqual = assert_equal(expectedAugmentedHessian,
      actualAugmentedHessian, 1e-6);

  //bool linEqual = assert_equal(expected2AugmentedHessian, actualAugmentedHessian, 1e-6);

  bool nonlinEqual = assert_equal(expected3AugmentedHessian,
      actualAugmentedHessian, 1e-6);

  //bool linCorrect = assert_equal(expected3AugmentedHessian, expected2AugmentedHessian, 1e-6);

  //actual2AugmentedHessian.bottomRightCorner(1,1) = expected3AugmentedHessian.bottomRightCorner(1,1); bool afterLinCorrect = assert_equal(expected3AugmentedHessian, actual2AugmentedHessian, 1e-6);

  actual3AugmentedHessian.bottomRightCorner(1, 1) =
      expected3AugmentedHessian.bottomRightCorner(1, 1);
  bool afterNonlinCorrect = assert_equal(expected3AugmentedHessian,
      actual3AugmentedHessian, 1e-6);

  bool ok = treeEqual && /*linEqual &&*/nonlinEqual
      && /*linCorrect &&*//*afterLinCorrect &&*/afterNonlinCorrect;

  return ok;
}

/* ************************************************************************* */
FixedLagSmoother::Result IncrementalFixedLagSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta, const KeyTimestampMap& timestamps) {

  const bool debug = ISDEBUG("IncrementalFixedLagSmoother update");

  if(debug) {
    std::cout << "IncrementalFixedLagSmoother::update() Start" << std::endl;
    PrintSymbolicTree(isam_, "Bayes Tree Before Update:");
    std::cout << "END" << std::endl;
  }

  FastVector<size_t> removedFactors;
  boost::optional<FastMap<Key,int> > constrainedKeys = boost::none;

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();

  if(debug) std::cout << "Current Timestamp: " << current_timestamp << std::endl;

  // Find the set of variables to be marginalized out
  std::set<Key> marginalizableKeys = findKeysBefore(current_timestamp - smootherLag_);

  if(debug) {
    std::cout << "Marginalizable Keys: ";
    BOOST_FOREACH(Key key, marginalizableKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Force iSAM2 to put the marginalizable variables at the beginning
  createOrderingConstraints(marginalizableKeys, constrainedKeys);

  if(debug) {
    std::cout << "Constrained Keys: ";
    if(constrainedKeys) {
      for(FastMap<Key,int>::const_iterator iter = constrainedKeys->begin(); iter != constrainedKeys->end(); ++iter) {
        std::cout << DefaultKeyFormatter(iter->first) << "(" << iter->second << ")  ";
      }
    }
    std::cout << std::endl;
  }

  // Update iSAM2
  ISAM2Result isamResult = isam_.update(newFactors, newTheta, FastVector<size_t>(), constrainedKeys);

  if(debug) {
    PrintSymbolicTree(isam_, "Bayes Tree After Update, Before Marginalization:");
    std::cout << "END" << std::endl;
  }

  // Marginalize out any needed variables
  FastList<Key> leafKeys(marginalizableKeys.begin(), marginalizableKeys.end());
//  isam_.experimentalMarginalizeLeaves(leafKeys);
  checkMarginalizeLeaves(isam_, leafKeys);

  // Remove marginalized keys from the KeyTimestampMap
  eraseKeyTimestampMap(marginalizableKeys);

  if(debug) {
    PrintSymbolicTree(isam_, "Final Bayes Tree:");
    std::cout << "END" << std::endl;
  }

  // TODO: Fill in result structure
  Result result;
  result.iterations = 1;
  result.linearVariables = 0;
  result.nonlinearVariables = 0;
  result.error = 0;

  if(debug) std::cout << "IncrementalFixedLagSmoother::update() Finish" << std::endl;

  return result;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::eraseKeysBefore(double timestamp) {
  TimestampKeyMap::iterator end = timestampKeyMap_.lower_bound(timestamp);
  TimestampKeyMap::iterator iter = timestampKeyMap_.begin();
  while(iter != end) {
    keyTimestampMap_.erase(iter->second);
    timestampKeyMap_.erase(iter++);
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::createOrderingConstraints(const std::set<Key>& marginalizableKeys, boost::optional<FastMap<Key,int> >& constrainedKeys) const {
  if(marginalizableKeys.size() > 0) {
    constrainedKeys = FastMap<Key,int>();
    // Generate ordering constraints so that the marginalizable variables will be eliminated first
    // Set all variables to Group1
    BOOST_FOREACH(const TimestampKeyMap::value_type& timestamp_key, timestampKeyMap_) {
      constrainedKeys->operator[](timestamp_key.second) = 1;
    }
    // Set marginalizable variables to Group0
    BOOST_FOREACH(Key key, marginalizableKeys){
      constrainedKeys->operator[](key) = 0;
    }
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintKeySet(const std::set<Key>& keys, const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor, const gtsam::Ordering& ordering) {
  std::cout << "f(";
  BOOST_FOREACH(Index index, factor->keys()) {
    std::cout << " " << index << "[" << gtsam::DefaultKeyFormatter(ordering.key(index)) << "]";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraph& graph, const gtsam::Ordering& ordering, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor, ordering);
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicTree(const gtsam::ISAM2& isam, const std::string& label) {
  std::cout << label << std::endl;
  if(isam.root())
    PrintSymbolicTreeHelper(isam.root(), isam.getOrdering());
  else
    std::cout << "{Empty Tree}" << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicTreeHelper(const gtsam::ISAM2Clique::shared_ptr& clique, const gtsam::Ordering& ordering, const std::string indent) {

  // Print the current clique
  std::cout << indent << "P( ";
  BOOST_FOREACH(gtsam::Index index, clique->conditional()->frontals()) {
    std::cout << gtsam::DefaultKeyFormatter(ordering.key(index)) << " ";
  }
  if(clique->conditional()->nrParents() > 0) std::cout << "| ";
  BOOST_FOREACH(gtsam::Index index, clique->conditional()->parents()) {
    std::cout << gtsam::DefaultKeyFormatter(ordering.key(index)) << " ";
  }
  std::cout << ")" << std::endl;

  // Recursively print all of the children
  BOOST_FOREACH(const gtsam::ISAM2Clique::shared_ptr& child, clique->children()) {
    PrintSymbolicTreeHelper(child, ordering, indent+" ");
  }
}

/* ************************************************************************* */
} /// namespace gtsam
