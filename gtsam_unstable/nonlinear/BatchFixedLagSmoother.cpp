/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchFixedLagSmoother.cpp
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/LinearizedFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/inference/inference.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void BatchFixedLagSmoother::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  FixedLagSmoother::print(s, keyFormatter);
  // TODO: What else to print?
}

/* ************************************************************************* */
bool BatchFixedLagSmoother::equals(const FixedLagSmoother& rhs, double tol) const {
  const BatchFixedLagSmoother* e =  dynamic_cast<const BatchFixedLagSmoother*> (&rhs);
  return e != NULL
      && FixedLagSmoother::equals(*e, tol)
      && factors_.equals(e->factors_, tol)
      && theta_.equals(e->theta_, tol);
}

/* ************************************************************************* */
FixedLagSmoother::Result BatchFixedLagSmoother::update(const NonlinearFactorGraph& newFactors, const Values& newTheta, const KeyTimestampMap& timestamps) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother update");
  if(debug) {
    std::cout << "BatchFixedLagSmoother::update() START" << std::endl;
  }

  // Add the new factors
  updateFactors(newFactors);

  // Add the new variables
  theta_.insert(newTheta);

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

  // Marginalize out these variables.
  // This removes any factors that touch marginalized variables and adds new linear(ized) factors to the graph
  marginalizeKeys(marginalizableKeys);

  // Create the optimizer
  Values linpoint;
  linpoint.insert(theta_);
  if(enforceConsistency_ && linearizedKeys_.size() > 0) {
    linpoint.update(linearizedKeys_);
  }
  LevenbergMarquardtOptimizer optimizer(factors_, linpoint, parameters_);

  // Use a custom optimization loop so the linearization points can be controlled
  double currentError;
  do {
    // Do next iteration
    currentError = optimizer.error();
    optimizer.iterate();

    // Force variables associated with linearized factors to keep the same linearization point
    if(enforceConsistency_ && linearizedKeys_.size() > 0) {
      // Put the old values of the linearized keys back into the optimizer state
      optimizer.state().values.update(linearizedKeys_);
      optimizer.state().error = factors_.error(optimizer.state().values);
    }

    // Maybe show output
    if(parameters_.verbosity >= NonlinearOptimizerParams::VALUES) optimizer.values().print("newValues");
    if(parameters_.verbosity >= NonlinearOptimizerParams::ERROR) std::cout << "newError: " << optimizer.error() << std::endl;
  } while(optimizer.iterations() < parameters_.maxIterations &&
      !checkConvergence(parameters_.relativeErrorTol, parameters_.absoluteErrorTol,
          parameters_.errorTol, currentError, optimizer.error(), parameters_.verbosity));

  // Update the Values from the optimizer
  theta_ = optimizer.values();

  // Create result structure
  Result result;
  result.iterations = optimizer.state().iterations;
  result.linearVariables = linearizedKeys_.size();
  result.nonlinearVariables = theta_.size() - linearizedKeys_.size();
  result.error = optimizer.state().error;

  if(debug) {
    std::cout << "BatchFixedLagSmoother::update() FINISH" << std::endl;
  }

  return result;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::updateFactors(const NonlinearFactorGraph& newFactors) {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, newFactors) {
    Index index;
    // Insert the factor into an existing hole in the factor graph, if possible
    if(availableSlots_.size() > 0) {
      index = availableSlots_.front();
      availableSlots_.pop();
      factors_.replace(index, factor);
    } else {
      index = factors_.size();
      factors_.push_back(factor);
    }
    // Update the FactorIndex
    BOOST_FOREACH(Key key, *factor) {
      factorIndex_[key].insert(index);
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::removeFactors(const std::set<size_t>& deleteFactors) {
  BOOST_FOREACH(size_t slot, deleteFactors) {
    if(factors_.at(slot)) {
      // Remove references to this factor from the FactorIndex
      BOOST_FOREACH(Key key, *(factors_.at(slot))) {
        factorIndex_[key].erase(slot);
      }
      // Remove the factor from the factor graph
      factors_.remove(slot);
      // Add the factor's old slot to the list of available slots
      availableSlots_.push(slot);
    } else {
      // TODO: Throw an error??
      std::cout << "Attempting to remove a factor from slot " << slot << ", but it is already NULL." << std::endl;
    }
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::eraseKeys(const std::set<Key>& keys) {

  BOOST_FOREACH(Key key, keys) {
    // Erase the key from the values
    theta_.erase(key);

    // Erase the key from the factor index
    factorIndex_.erase(key);

    // Erase the key from the set of linearized keys
    if(linearizedKeys_.exists(key)) {
      linearizedKeys_.erase(key);
    }
  }

  eraseKeyTimestampMap(keys);
}

/* ************************************************************************* */
struct FactorTree {
  std::set<Index> factors;
  std::set<Key> keys;

  FactorTree(const std::set<Index>& factors, const NonlinearFactorGraph& allFactors) : factors(factors) {
    BOOST_FOREACH(const Index& factor, factors) {
      BOOST_FOREACH(Key key, *(allFactors.at(factor))) {
        keys.insert(key);
      }
    }
  };

  void push_back(const FactorTree& factorTree) {
    factors.insert(factorTree.factors.begin(), factorTree.factors.end());
    keys.insert(factorTree.keys.begin(), factorTree.keys.end());
  }

  bool hasCommonKeys(Index factor, const NonlinearFactorGraph& allFactors) {
    const NonlinearFactor::shared_ptr& f = allFactors.at(factor);
    std::set<Key>::const_iterator iter = std::find_first_of(keys.begin(), keys.end(), f->begin(), f->end());
    return iter != keys.end();
  }

  template <class ForwardIterator>
  bool hasCommonKeys(ForwardIterator first, ForwardIterator last, const NonlinearFactorGraph& allFactors) {
    for(ForwardIterator factor = first; factor != last; ++factor) {
      if(hasCommonKeys(*factor, allFactors))
        return true;
    }
    return false;
  }
};

/* ************************************************************************* */
void BatchFixedLagSmoother::marginalizeKeys(const std::set<Key>& marginalizableKeys) {

  const bool debug = ISDEBUG("BatchFixedLagSmoother update");
  if(debug) std::cout << "BatchFixedLagSmoother::marginalizeKeys() START" << std::endl;


  // In order to marginalize out the selected variables, the factors involved in those variables
  // must be identified and removed from iSAM2. Also, the effect of those removed factors on the
  // remaining variables needs to be accounted for. This will be done with linear(ized) factors from
  // a partial clique marginalization (or from the iSAM2 cached factor if the entire clique is removed).
  // This function finds the set of factors to be removed and generates the linearized factors that
  // must be added.

  if(marginalizableKeys.size() > 0) {

    if(debug) PrintKeySet(marginalizableKeys, "Marginalizable Keys:");

    // Find all of the factors associated with marginalizable variables. This set of factors may form a forest.
    typedef std::list<FactorTree> FactorForest;
    FactorForest factorForest;
    BOOST_FOREACH(Key key, marginalizableKeys) {

      if(debug) std::cout << "Looking for factors involving key " << DefaultKeyFormatter(key) << std::endl;

      // Get the factors associated with this variable
      const std::set<size_t>& factors = factorIndex_.at(key);

      if(debug) { std::cout << "Found the following factors:" << std::endl; BOOST_FOREACH(size_t i, factors) { std::cout << "   "; PrintSymbolicFactor(factors_.at(i)); } }

      // Loop over existing factor trees, looking for common keys
      std::vector<FactorForest::iterator> commonTrees;
      for(FactorForest::iterator tree = factorForest.begin(); tree != factorForest.end(); ++tree) {
        if(tree->hasCommonKeys(factors.begin(), factors.end(), factors_)) {
          commonTrees.push_back(tree);
        }
      }

      if(debug) std::cout << "Found " << commonTrees.size() << " common trees." << std::endl;

      if(commonTrees.size() == 0) {
        // No common trees were found. Create a new one.
        factorForest.push_back(FactorTree(factors, factors_));

        if(debug) std::cout << "Created a new tree." << std::endl;

      } else {
        // Extract the last common tree
        FactorForest::iterator commonTree = commonTrees.back();
        commonTrees.pop_back();
        // Merge the current factors into this tree
        commonTree->push_back(FactorTree(factors, factors_));
        // Merge all other common trees into this one, deleting the other trees from the forest.
        BOOST_FOREACH(FactorForest::iterator& tree, commonTrees) {
          commonTree->push_back(*tree);
          factorForest.erase(tree);
        }
      }
    }

    if(debug) std::cout << "Found " << factorForest.size() << " factor trees in the set of removed factors." << std::endl;

    // For each tree in the forest:
    //  (0) construct an ordering for the tree
    //  (1) construct a linear factor graph
    //  (2) solve for the marginal factors
    //  (3) convert the marginal factors into Linearized Factors
    //  (4) remove the marginalized factors from the graph
    //  (5) add the factors in this tree to the graph
    BOOST_FOREACH(const FactorTree& factorTree, factorForest) {
      //  (0) construct an ordering for this tree
      //      The ordering should place the marginalizable keys first, then the remaining keys
      Ordering ordering;

      std::set<Key> marginalizableTreeKeys;
      std::set_intersection(factorTree.keys.begin(), factorTree.keys.end(),
                            marginalizableKeys.begin(), marginalizableKeys.end(),
                            std::inserter(marginalizableTreeKeys, marginalizableTreeKeys.end()));

      std::set<Key> remainingTreeKeys;
      std::set_difference(factorTree.keys.begin(), factorTree.keys.end(),
                          marginalizableTreeKeys.begin(), marginalizableTreeKeys.end(),
                          std::inserter(remainingTreeKeys, remainingTreeKeys.end()));

      // TODO: It may be worthwhile to use CCOLAMD here. (but maybe not???)
      BOOST_FOREACH(Key key, marginalizableTreeKeys) {
        ordering.push_back(key);
      }
      BOOST_FOREACH(Key key, remainingTreeKeys) {
        ordering.push_back(key);
      }

      //  (1) construct a linear factor graph
      GaussianFactorGraph graph;
      BOOST_FOREACH(size_t factor, factorTree.factors) {
        graph.push_back( factors_.at(factor)->linearize(theta_, ordering) );
      }

      if(debug) PrintSymbolicGraph(graph, ordering, "Factor Tree:");

      //  (2) solve for the marginal factors
      // Perform partial elimination, resulting in a conditional probability ( P(MarginalizedVariable | RemainingVariables)
      // and factors on the remaining variables ( f(RemainingVariables) ). These are the factors we need to add to iSAM2
      std::vector<Index> variables;
      BOOST_FOREACH(Key key, marginalizableTreeKeys) {
        variables.push_back(ordering.at(key));
      }
      std::pair<GaussianFactorGraph::sharedConditional, GaussianFactorGraph> result = graph.eliminate(variables);
      graph = result.second;

      if(debug) PrintSymbolicGraph(graph, ordering, "Factors on Remaining Variables:");

      //  (3) convert the marginal factors into Linearized Factors
      NonlinearFactorGraph newFactors;
      BOOST_FOREACH(const GaussianFactor::shared_ptr& gaussianFactor, graph) {
        // These factors are all generated from BayesNet conditionals. They should all be Jacobians.
        JacobianFactor::shared_ptr jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(gaussianFactor);
        assert(jacobianFactor);
        LinearizedGaussianFactor::shared_ptr factor = LinearizedJacobianFactor::shared_ptr(new LinearizedJacobianFactor(jacobianFactor, ordering, theta_));
        // add it to the new factor set
        newFactors.push_back(factor);
      }

      //  (4) remove the marginalized factors from the graph
      removeFactors(factorTree.factors);

      //  (5) add the factors in this tree to the main set of factors
      updateFactors(newFactors);

      //  (6) add the keys involved in the linear(ized) factors to the linearizedKey list
      FastSet<Key> linearizedKeys = newFactors.keys();
      BOOST_FOREACH(Key key, linearizedKeys) {
        if(!linearizedKeys_.exists(key)) {
          linearizedKeys_.insert(key, theta_.at(key));
        }
      }
    }

    // Remove the marginalized keys from the smoother data structures
    eraseKeys(marginalizableKeys);
  }

  if(debug) std::cout << "BatchFixedLagSmoother::marginalizeKeys() FINISH" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintKeySet(const std::set<Key>& keys, const std::string& label) {
  std::cout << label;
  BOOST_FOREACH(gtsam::Key key, keys) {
    std::cout << " " << gtsam::DefaultKeyFormatter(key);
  }
  std::cout << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(const NonlinearFactor::shared_ptr& factor) {
  std::cout << "f(";
  if(factor) {
    BOOST_FOREACH(Key key, factor->keys()) {
      std::cout << " " << gtsam::DefaultKeyFormatter(key);
    }
  } else {
    std::cout << " NULL";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicFactor(const GaussianFactor::shared_ptr& factor, const Ordering& ordering) {
  std::cout << "f(";
  BOOST_FOREACH(Index index, factor->keys()) {
    std::cout << " " << index << "[" << gtsam::DefaultKeyFormatter(ordering.key(index)) << "]";
  }
  std::cout << " )" << std::endl;
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const NonlinearFactorGraph& graph, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor);
  }
}

/* ************************************************************************* */
void BatchFixedLagSmoother::PrintSymbolicGraph(const GaussianFactorGraph& graph, const Ordering& ordering, const std::string& label) {
  std::cout << label << std::endl;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, graph) {
    PrintSymbolicFactor(factor, ordering);
  }
}

/* ************************************************************************* */
} /// namespace gtsam
