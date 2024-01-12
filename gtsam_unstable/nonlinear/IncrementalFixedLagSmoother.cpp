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

namespace gtsam {

/* ************************************************************************* */
void recursiveMarkAffectedKeys(const Key& key,
    const ISAM2Clique::shared_ptr& clique, std::set<Key>& additionalKeys) {

  // Check if the separator keys of the current clique contain the specified key
  if (std::find(clique->conditional()->beginParents(),
      clique->conditional()->endParents(), key)
      != clique->conditional()->endParents()) {

    // Mark the frontal keys of the current clique
    for(Key i: clique->conditional()->frontals()) {
      additionalKeys.insert(i);
    }

    // Recursively mark all of the children
    for(const ISAM2Clique::shared_ptr& child: clique->children) {
      recursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  // If the key was not found in the separator/parents, then none of its children can have it either
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool IncrementalFixedLagSmoother::equals(const FixedLagSmoother& rhs,
    double tol) const {
  const IncrementalFixedLagSmoother* e =
      dynamic_cast<const IncrementalFixedLagSmoother*>(&rhs);
  return e != nullptr && FixedLagSmoother::equals(*e, tol)
      && isam_.equals(e->isam_, tol);
}

/* ************************************************************************* */
FixedLagSmoother::Result IncrementalFixedLagSmoother::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const KeyTimestampMap& timestamps, const FactorIndices& factorsToRemove) {

  const bool debug = ISDEBUG("IncrementalFixedLagSmoother update");

  if (debug) {
    std::cout << "IncrementalFixedLagSmoother::update() Start" << std::endl;
    PrintSymbolicTree(isam_, "Bayes Tree Before Update:");
    std::cout << "END" << std::endl;
  }

  FastVector<size_t> removedFactors;
  std::optional<FastMap<Key, int> > constrainedKeys = {};

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();

  if (debug)
    std::cout << "Current Timestamp: " << current_timestamp << std::endl;

  // Find the set of variables to be marginalized out
  KeyVector marginalizableKeys = findKeysBefore(
      current_timestamp - smootherLag_);

  if (debug) {
    std::cout << "Marginalizable Keys: ";
    for(Key key: marginalizableKeys) {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Force iSAM2 to put the marginalizable variables at the beginning
  createOrderingConstraints(marginalizableKeys, constrainedKeys);

  if (debug) {
    std::cout << "Constrained Keys: ";
    if (constrainedKeys) {
      for (FastMap<Key, int>::const_iterator iter = constrainedKeys->begin();
          iter != constrainedKeys->end(); ++iter) {
        std::cout << DefaultKeyFormatter(iter->first) << "(" << iter->second
            << ")  ";
      }
    }
    std::cout << std::endl;
  }

  // Mark additional keys between the marginalized keys and the leaves
  std::set<Key> additionalKeys;
  for(Key key: marginalizableKeys) {
    ISAM2Clique::shared_ptr clique = isam_[key];
    for(const ISAM2Clique::shared_ptr& child: clique->children) {
      recursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

  // Update iSAM2
  isamResult_ = isam_.update(newFactors, newTheta,
      factorsToRemove, constrainedKeys, {}, additionalMarkedKeys);

  if (debug) {
    PrintSymbolicTree(isam_,
        "Bayes Tree After Update, Before Marginalization:");
    std::cout << "END" << std::endl;
  }

  // Marginalize out any needed variables
  if (marginalizableKeys.size() > 0) {
    FastList<Key> leafKeys(marginalizableKeys.begin(),
        marginalizableKeys.end());
    isam_.marginalizeLeaves(leafKeys);
  }

  // Remove marginalized keys from the KeyTimestampMap
  eraseKeyTimestampMap(marginalizableKeys);

  if (debug) {
    PrintSymbolicTree(isam_, "Final Bayes Tree:");
    std::cout << "END" << std::endl;
  }

  // TODO: Fill in result structure
  Result result;
  result.iterations = 1;
  result.linearVariables = 0;
  result.nonlinearVariables = 0;
  result.error = 0;

  if (debug)
    std::cout << "IncrementalFixedLagSmoother::update() Finish" << std::endl;

  return result;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::eraseKeysBefore(double timestamp) {
  TimestampKeyMap::iterator end = timestampKeyMap_.lower_bound(timestamp);
  TimestampKeyMap::iterator iter = timestampKeyMap_.begin();
  while (iter != end) {
    keyTimestampMap_.erase(iter->second);
    timestampKeyMap_.erase(iter++);
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::createOrderingConstraints(
    const KeyVector& marginalizableKeys,
    std::optional<FastMap<Key, int> >& constrainedKeys) const {
  if (marginalizableKeys.size() > 0) {
    constrainedKeys = FastMap<Key, int>();
    // Generate ordering constraints so that the marginalizable variables will be eliminated first
    // Set all variables to Group1
    for(const TimestampKeyMap::value_type& timestamp_key: timestampKeyMap_) {
      constrainedKeys->operator[](timestamp_key.second) = 1;
    }
    // Set marginalizable variables to Group0
    for(Key key: marginalizableKeys) {
      constrainedKeys->operator[](key) = 0;
    }
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintKeySet(const std::set<Key>& keys,
    const std::string& label) {
  std::cout << label;
  for(Key key: keys) {
    std::cout << " " << DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicFactor(
    const GaussianFactor::shared_ptr& factor) {
  std::cout << "f(";
  for(Key key: factor->keys()) {
    std::cout << " " << DefaultKeyFormatter(key);
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicGraph(
    const GaussianFactorGraph& graph, const std::string& label) {
  std::cout << label << std::endl;
  for(const GaussianFactor::shared_ptr& factor: graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicTree(const ISAM2& isam,
    const std::string& label) {
  std::cout << label << std::endl;
  if (!isam.roots().empty()) {
    for(const ISAM2::sharedClique& root: isam.roots()) {
      PrintSymbolicTreeHelper(root);
    }
  } else
    std::cout << "{Empty Tree}" << std::endl;
}

/* ************************************************************************* */
void IncrementalFixedLagSmoother::PrintSymbolicTreeHelper(
    const ISAM2Clique::shared_ptr& clique, const std::string indent) {

  // Print the current clique
  std::cout << indent << "P( ";
  for(Key key: clique->conditional()->frontals()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  if (clique->conditional()->nrParents() > 0)
    std::cout << "| ";
  for(Key key: clique->conditional()->parents()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  std::cout << ")" << std::endl;

  // Recursively print all of the children
  for(const ISAM2Clique::shared_ptr& child: clique->children) {
    PrintSymbolicTreeHelper(child, indent + " ");
  }
}

/* ************************************************************************* */
} /// namespace gtsam
