/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentBatchFilter.cpp
 * @brief   A Levenberg-Marquardt Batch Filter that implements the
 *          Concurrent Filtering and Smoothing interface.
 * @author  Stephen Williams
 */
#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/debug.h>

namespace gtsam {

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintNonlinearFactor(const NonlinearFactor::shared_ptr& factor,
    const std::string& indent, const KeyFormatter& keyFormatter) {
  std::cout << indent;
  if(factor) {
    if(boost::dynamic_pointer_cast<LinearContainerFactor>(factor)) {
      std::cout << "l( ";
    } else {
      std::cout << "f( ";
    }
    for(Key key: *factor) {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << ")" << std::endl;
  } else {
    std::cout << "{ nullptr }" << std::endl;
  }
}

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintNonlinearFactorGraph(const NonlinearFactorGraph& factors,
    const std::string& indent, const std::string& title, const KeyFormatter& keyFormatter) {
  std::cout << indent << title << std::endl;
  for(const NonlinearFactor::shared_ptr& factor: factors) {
    PrintNonlinearFactor(factor, indent + "  ", keyFormatter);
  }
}

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintNonlinearFactorGraph(const NonlinearFactorGraph& factors, const std::vector<size_t>& slots,
    const std::string& indent, const std::string& title, const KeyFormatter& keyFormatter) {
  std::cout << indent << title << std::endl;
  for(size_t slot: slots) {
    PrintNonlinearFactor(factors.at(slot), indent + "  ", keyFormatter);
  }
}

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintLinearFactor(const GaussianFactor::shared_ptr& factor,
    const std::string& indent, const KeyFormatter& keyFormatter) {
  std::cout << indent;
  if(factor) {
    JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(factor);
    HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(factor);
    if(jf) {
      std::cout << "j( ";
    } else if(hf) {
      std::cout << "h( ";
    } else {
      std::cout << "g( ";
    }
    for(Key key: *factor) {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << ")" << std::endl;
  } else {
    std::cout << "{ nullptr }" << std::endl;
  }
}

/* ************************************************************************* */
void ConcurrentBatchFilter::PrintLinearFactorGraph(const GaussianFactorGraph& factors,
    const std::string& indent, const std::string& title, const KeyFormatter& keyFormatter) {
  std::cout << indent << title << std::endl;
  for(const GaussianFactor::shared_ptr& factor: factors) {
    PrintLinearFactor(factor, indent + "  ", keyFormatter);
  }
}

/* ************************************************************************* */
void ConcurrentBatchFilter::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s;
  PrintNonlinearFactorGraph(factors_, "  ", "Factors:");
  PrintKeys(theta_.keys(), "  ", "Values:");
  PrintNonlinearFactorGraph(smootherFactors_, "  ", "Cached Smoother Factors:");
  PrintKeys(smootherValues_.keys(), "  ", "Cached Smoother Values:");
}

/* ************************************************************************* */
bool ConcurrentBatchFilter::equals(const ConcurrentFilter& rhs, double tol) const {
  const ConcurrentBatchFilter* filter = dynamic_cast<const ConcurrentBatchFilter*>(&rhs);
  return filter
      && factors_.equals(filter->factors_)
      && theta_.equals(filter->theta_)
      && ordering_.equals(filter->ordering_)
      && delta_.equals(filter->delta_)
      && separatorValues_.equals(filter->separatorValues_)
      && smootherSummarization_.equals(filter->smootherSummarization_)
      && smootherShortcut_.equals(filter->smootherShortcut_)
      && filterSummarization_.equals(filter->filterSummarization_)
      && smootherFactors_.equals(filter->smootherFactors_)
      && smootherValues_.equals(filter->smootherValues_);
}

/* ************************************************************************* */
ConcurrentBatchFilter::Result ConcurrentBatchFilter::update(const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<FastList<Key> >& keysToMove, const std::optional< std::vector<size_t> >& removeFactorIndices) {

  gttic(update);

//  const bool debug = ISDEBUG("ConcurrentBatchFilter update");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentBatchFilter::update  Begin" << std::endl;

  // Create the return result meta-data
  Result result;

  if(debug) std::cout << "ConcurrentBatchFilter::update  Augmenting System ..." << std::endl;

  // Update all of the internal variables with the new information
  gttic(augment_system);

  // Add the new variables to theta
  theta_.insert(newTheta);
  // Add new variables to the end of the ordering
  for (const auto key : newTheta.keys()) {
    ordering_.push_back(key);
  }
  // Augment Delta
  delta_.insert(newTheta.zeroVectors());

  // Add the new factors to the graph, updating the variable index
  result.newFactorsIndices = insertFactors(newFactors);

  if(removeFactorIndices){
    if(debug){
      std::cout << "ConcurrentBatchFilter::update  removeFactorIndices " << std::endl;
    }
    removeFactors(*removeFactorIndices);
  }

  gttoc(augment_system);

  if(debug) std::cout << "ConcurrentBatchFilter::update  Reordering System ..." << std::endl;

  // Reorder the system to ensure efficient optimization (and marginalization) performance
  gttic(reorder);
  reorder(keysToMove);
  gttoc(reorder);

  if(debug) std::cout << "ConcurrentBatchFilter::update  Optimizing System ..." << std::endl;

  // Optimize the factors using a modified version of L-M
  gttic(optimize);
  if(factors_.size() > 0) {
    optimize(factors_, theta_, ordering_, delta_, separatorValues_, parameters_, result);
  }
  gttoc(optimize);

  if(debug) std::cout << "ConcurrentBatchFilter::update  Moving Separator ..." << std::endl;

  gttic(move_separator);
  if(keysToMove && keysToMove->size() > 0){
    moveSeparator(*keysToMove);
  }
  gttoc(move_separator);

  if(debug) std::cout << "ConcurrentBatchFilter::update  End" << std::endl;

  gttoc(update);

  return result;
}

/* ************************************************************************* */
void ConcurrentBatchFilter::presync() {

  gttic(presync);

  gttoc(presync);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::synchronize(const NonlinearFactorGraph& smootherSummarization, const Values& smootherSummarizationValues) {

  gttic(synchronize);

//  const bool debug = ISDEBUG("ConcurrentBatchFilter synchronize");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentBatchFilter::synchronize  Begin" << std::endl;

  if(debug) { PrintNonlinearFactorGraph(smootherSummarization_, "ConcurrentBatchFilter::synchronize  ", "Previous Smoother Summarization:", DefaultKeyFormatter); }

#ifndef NDEBUG
  std::set<Key> oldKeys = smootherSummarization_.keys();
  std::set<Key> newKeys = smootherSummarization.keys();
  assert(oldKeys.size() == newKeys.size());
  assert(std::equal(oldKeys.begin(), oldKeys.end(), newKeys.begin()));
#endif

  // Update the smoother summarization on the old separator
  smootherSummarization_ = smootherSummarization;

  if(debug) { PrintNonlinearFactorGraph(smootherSummarization_, "ConcurrentBatchFilter::synchronize  ", "Updated Smoother Summarization:", DefaultKeyFormatter); }

  // Find the set of new separator keys
  const KeySet newSeparatorKeys = separatorValues_.keySet();

  if(debug) { PrintKeys(newSeparatorKeys, "ConcurrentBatchFilter::synchronize  ", "Current Separator Keys:"); }

  // Use the shortcut to calculate an updated marginal on the current separator
  {
    // Combine just the shortcut and the previousSmootherSummarization
    NonlinearFactorGraph graph;
    graph.push_back(smootherSummarization_);
    graph.push_back(smootherShortcut_);
    Values values;
    values.insert(smootherSummarizationValues);
    for(const auto key: newSeparatorKeys) {
      if(!values.exists(key)) {
        values.insert(key, separatorValues_.at(key));
      }
    }
    // Calculate the summarized factor on just the new separator keys
    smootherSummarization_ = internal::calculateMarginalFactors(graph, values, newSeparatorKeys, parameters_.getEliminationFunction());

    // Remove the old factors on the separator and insert new
    removeFactors(separatorSummarizationSlots_);
    separatorSummarizationSlots_ = insertFactors(smootherSummarization_);

    // Clear the smoother shortcut
    smootherShortcut_.resize(0);
  }

  if(debug) { PrintNonlinearFactorGraph(factors_, separatorSummarizationSlots_, "ConcurrentBatchFilter::synchronize  ", "Current Separator Summarization:", DefaultKeyFormatter); }

  // Calculate the marginal on the new separator from the filter factors
  // Note: This could also be done during each filter update so it would simply be available
  {
    // Calculate the summarized factor on just the new separator keys (from the filter side)
    // Copy all of the factors from the filter, not including the smoother summarization
    NonlinearFactorGraph factors;
    for(size_t slot = 0; slot < factors_.size(); ++slot) {
      if(factors_.at(slot) && std::find(separatorSummarizationSlots_.begin(), separatorSummarizationSlots_.end(), slot) == separatorSummarizationSlots_.end()) {
        factors.push_back(factors_.at(slot));
      }
    }
    filterSummarization_ = internal::calculateMarginalFactors(factors, theta_, newSeparatorKeys, parameters_.getEliminationFunction());
  }

  if(debug) { PrintNonlinearFactorGraph(filterSummarization_, "ConcurrentBatchFilter::synchronize  ", "Updated Filter Summarization:", DefaultKeyFormatter); }

  if(debug) std::cout << "ConcurrentBatchFilter::synchronize  End" << std::endl;

  gttoc(synchronize);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::getSummarizedFactors(NonlinearFactorGraph& filterSummarization, Values& filterSummarizationValues) {

  gttic(get_summarized_factors);

  // Copy the previous calculated smoother summarization factors into the output
  filterSummarization.push_back(filterSummarization_);
  filterSummarizationValues.insert(separatorValues_);

  gttoc(get_summarized_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::getSmootherFactors(NonlinearFactorGraph& smootherFactors, Values& smootherValues) {

  gttic(get_smoother_factors);

  // Copy the previous calculated smoother summarization factors into the output
  smootherFactors.push_back(smootherFactors_);
  smootherValues.insert(smootherValues_);

  gttoc(get_smoother_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::postsync() {

  gttic(postsync);

  // Clear out the factors and values that were just sent to the smoother
  smootherFactors_.resize(0);
  smootherValues_.clear();

  gttoc(postsync);
}

/* ************************************************************************* */
std::vector<size_t> ConcurrentBatchFilter::insertFactors(const NonlinearFactorGraph& factors) {

  gttic(insert_factors);

  // create the output vector
  std::vector<size_t> slots;
  slots.reserve(factors.size());

  // Insert the factor into an existing hole in the factor graph, if possible
  for(const NonlinearFactor::shared_ptr& factor: factors) {
    size_t slot;
    if(availableSlots_.size() > 0) {
      slot = availableSlots_.front();
      availableSlots_.pop();
      factors_.replace(slot, factor);
    } else {
      slot = factors_.size();
      factors_.push_back(factor);
    }
    slots.push_back(slot);
  }

  gttoc(insert_factors);

  return slots;
}

/* ************************************************************************* */
void ConcurrentBatchFilter::removeFactors(const std::vector<size_t>& slots) {

  gttic(remove_factors);

  // For each factor slot to delete...
  for(size_t slot: slots) {

    // Remove the factor from the graph
    factors_.remove(slot);

    // Mark the factor slot as available
    availableSlots_.push(slot);
  }

  gttoc(remove_factors);
}

/* ************************************************************************* */
void ConcurrentBatchFilter::reorder(const std::optional<FastList<Key> >& keysToMove) {

  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  if(keysToMove && keysToMove->size() > 0) {
    ordering_ = Ordering::ColamdConstrainedFirst(factors_, KeyVector(keysToMove->begin(), keysToMove->end()));
  }else{
    ordering_ = Ordering::Colamd(factors_);
  }

}

/* ************************************************************************* */
void ConcurrentBatchFilter::optimize(const NonlinearFactorGraph& factors, Values& theta, const Ordering& ordering,
     VectorValues& delta, const Values& linearValues, const LevenbergMarquardtParams& parameters,
     ConcurrentBatchFilter::Result& result) {

  //  const bool debug = ISDEBUG("ConcurrentBatchFilter optimize");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentBatchFilter::optimize  Begin" << std::endl;

  // Create output result structure
  result.nonlinearVariables = theta.size() - linearValues.size();
  result.linearVariables = linearValues.size();

  // Set optimization parameters
  double lambda = parameters.lambdaInitial;
  double lambdaFactor = parameters.lambdaFactor;
  double lambdaUpperBound = parameters.lambdaUpperBound;
  double lambdaLowerBound = 1.0e-10;
  size_t maxIterations = parameters.maxIterations;
  double relativeErrorTol = parameters.relativeErrorTol;
  double absoluteErrorTol = parameters.absoluteErrorTol;
  double errorTol = parameters.errorTol;

  // Create a Values that holds the current evaluation point
  Values evalpoint = theta.retract(delta);
  result.error = factors.error(evalpoint);

  // check if we're already close enough
  if(result.error <= errorTol) {
    if(debug) { std::cout << "Exiting, as error = " << result.error << " < " << errorTol << std::endl; }
  }

  if(debug) {
    std::cout << "linearValues: " << linearValues.size() << std::endl;
    std::cout << "Initial error: " << result.error << std::endl;
  }

  // Use a custom optimization loop so the linearization points can be controlled
  double previousError;
  VectorValues newDelta;
  do {
    previousError = result.error;

    // Do next iteration
    gttic(optimizer_iteration);

      // Linearize graph around the linearization point
      GaussianFactorGraph linearFactorGraph = *factors.linearize(theta);

      // Keep increasing lambda until we make make progress
      while(true) {

        if(debug) { std::cout << "trying lambda = " << lambda << std::endl; }

        // Add prior factors at the current solution
        gttic(damp);
        GaussianFactorGraph dampedFactorGraph(linearFactorGraph);
        dampedFactorGraph.reserve(linearFactorGraph.size() + delta.size());
        double sigma = 1.0 / std::sqrt(lambda);

        // for each of the variables, add a prior at the current solution
        for(const VectorValues::KeyValuePair& key_value: delta) {
          size_t dim = key_value.second.size();
          Matrix A = Matrix::Identity(dim,dim);
          Vector b = key_value.second;
          SharedDiagonal model = noiseModel::Isotropic::Sigma(dim, sigma);
          GaussianFactor::shared_ptr prior(new JacobianFactor(key_value.first, A, b, model));
          dampedFactorGraph.push_back(prior);
        }

        gttoc(damp);
        result.lambdas++;

        gttic(solve);
        // Solve Damped Gaussian Factor Graph
        newDelta = dampedFactorGraph.optimize(ordering, parameters.getEliminationFunction());
        // update the evalpoint with the new delta
        evalpoint = theta.retract(newDelta);
        gttoc(solve);

        // Evaluate the new nonlinear error
        gttic(compute_error);
        double error = factors.error(evalpoint);
        gttoc(compute_error);

        if(debug) {
          std::cout << "linear delta norm = " << newDelta.norm() << std::endl;
          std::cout << "next error = " << error << std::endl;
        }

        if(error < result.error) {
          // Keep this change
          // Update the error value
          result.error = error;
          // Update the linearization point
          theta = evalpoint;
          // Reset the deltas to zeros
          delta.setZero();
          // Put the linearization points and deltas back for specific variables
          if(linearValues.size() > 0) {
            theta.update(linearValues);
            for(const auto key: linearValues.keys()) {
              delta.at(key) = newDelta.at(key);
            }
          }

          // Decrease lambda for next time
          lambda /= lambdaFactor;
          if(lambda < lambdaLowerBound) {
            lambda = lambdaLowerBound;
          }
          // End this lambda search iteration
          break;
        } else {
          // Reject this change
          if(lambda >= lambdaUpperBound) {
            // The maximum lambda has been used. Print a warning and end the search.
            std::cout << "Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda" << std::endl;
            break;
          } else {
            // Increase lambda and continue searching
            lambda *= lambdaFactor;
          }
        }
      } // end while

    gttoc(optimizer_iteration);

    if(debug) { std::cout << "using lambda = " << lambda << std::endl; }

    result.iterations++;
  } while(result.iterations < maxIterations &&
      !checkConvergence(relativeErrorTol, absoluteErrorTol, errorTol, previousError, result.error, NonlinearOptimizerParams::SILENT));

  if(debug) { std::cout << "newError: " << result.error << std::endl; }

  if(debug) std::cout << "ConcurrentBatchFilter::optimize  End" << std::endl;
}

/* ************************************************************************* */
void ConcurrentBatchFilter::moveSeparator(const FastList<Key>& keysToMove) {
  // In order to move the separator, we need to calculate the marginal information on the new
  // separator from all of the factors on the smoother side (both the factors actually in the
  // smoother and the ones to be transitioned to the smoother but stored in the filter).
  // This is exactly the same operation that is performed by a fixed-lag smoother, calculating
  // a marginal factor from the variables outside the smoother window.
  //
  // However, for the concurrent system, we would like to calculate this marginal in a particular
  // way, such that an intermediate term is produced that provides a "shortcut" between the old
  // separator (as defined in the smoother) and the new separator. This will allow us to quickly
  // update the new separator with changes at the old separator (from the smoother)

  // TODO: This is currently not very efficient: multiple calls to graph.keys(), etc.

//  const bool debug = ISDEBUG("ConcurrentBatchFilter moveSeparator");
  const bool debug = false;

  if(debug) std::cout << "ConcurrentBatchFilter::moveSeparator  Begin" << std::endl;

  if(debug) { PrintKeys(keysToMove, "ConcurrentBatchFilter::moveSeparator  ", "Keys To Move:", DefaultKeyFormatter); }


  // Identify all of the new factors to be sent to the smoother (any factor involving keysToMove)
  std::vector<size_t> removedFactorSlots;
  VariableIndex variableIndex(factors_);
  for(Key key: keysToMove) {
    const auto& slots = variableIndex[key];
    removedFactorSlots.insert(removedFactorSlots.end(), slots.begin(), slots.end());
  }

  // Sort and remove duplicates
  std::sort(removedFactorSlots.begin(), removedFactorSlots.end());
  removedFactorSlots.erase(std::unique(removedFactorSlots.begin(), removedFactorSlots.end()), removedFactorSlots.end());
  // Remove any linear/marginal factor that made it into the set
  for(size_t index: separatorSummarizationSlots_) {
    removedFactorSlots.erase(std::remove(removedFactorSlots.begin(), removedFactorSlots.end(), index), removedFactorSlots.end());
  }

  // TODO: Make this compact
  if(debug) {
    std::cout << "ConcurrentBatchFilter::moveSeparator  Removed Factor Slots: ";
    for(size_t slot: removedFactorSlots) {
      std::cout << slot << " ";
    }
    std::cout << std::endl;
  }

  // Add these factors to a factor graph
  NonlinearFactorGraph removedFactors;
  for(size_t slot: removedFactorSlots) {
    if(factors_.at(slot)) {
      removedFactors.push_back(factors_.at(slot));
    }
  }

  if(debug) {
    PrintNonlinearFactorGraph(removedFactors, "ConcurrentBatchFilter::synchronize  ", "Removed Factors:", DefaultKeyFormatter);
    PrintNonlinearFactorGraph(smootherShortcut_, "ConcurrentBatchFilter::synchronize  ", "Old Shortcut:", DefaultKeyFormatter);
    PrintKeys(smootherShortcut_.keys(), "ConcurrentBatchFilter::moveSeparator  ", "Old Shortcut Keys:", DefaultKeyFormatter);
    PrintKeys(separatorValues_.keys(), "ConcurrentBatchFilter::moveSeparator  ", "Previous Separator Keys:", DefaultKeyFormatter);
  }

  // Calculate the set of new separator keys: AffectedKeys + PreviousSeparatorKeys - KeysToMove
  KeySet newSeparatorKeys = removedFactors.keys();
  newSeparatorKeys.merge(separatorValues_.keySet());
  for(Key key: keysToMove) {
    newSeparatorKeys.erase(key);
  }

  if(debug) { PrintKeys(newSeparatorKeys, "ConcurrentBatchFilter::synchronize  ", "New Separator Keys:", DefaultKeyFormatter); }

  // Calculate the set of shortcut keys: NewSeparatorKeys + OldSeparatorKeys
  KeySet shortcutKeys = newSeparatorKeys;
  for(Key key: smootherSummarization_.keys()) {
    shortcutKeys.insert(key);
  }

  if(debug) { PrintKeys(shortcutKeys, "ConcurrentBatchFilter::moveSeparator  ", "Old Shortcut Keys:", DefaultKeyFormatter); }

  // Calculate a new shortcut
  {
    // Combine the previous shortcut factor with all of the new factors being sent to the smoother
    NonlinearFactorGraph graph;
    graph.push_back(removedFactors);
    graph.push_back(smootherShortcut_);
    Values values;
    values.insert(smootherValues_);
    values.insert(theta_);
    // Calculate the summarized factor on the shortcut keys
    smootherShortcut_ = internal::calculateMarginalFactors(graph, values, shortcutKeys, parameters_.getEliminationFunction());
  }

  if(debug) {
    PrintNonlinearFactorGraph(smootherShortcut_, "ConcurrentBatchFilter::synchronize  ", "New Shortcut:", DefaultKeyFormatter);
    PrintNonlinearFactorGraph(smootherSummarization_, "ConcurrentBatchFilter::synchronize  ", "Smoother Summarization:", DefaultKeyFormatter);
    PrintNonlinearFactorGraph(factors_, separatorSummarizationSlots_, "ConcurrentBatchFilter::synchronize  ", "Current Separator Summarization:", DefaultKeyFormatter);
  }

  // Calculate the new smoother summarization on the new separator using the shortcut
  NonlinearFactorGraph separatorSummarization;
  {
    // Combine just the shortcut and the previousSmootherSummarization
    NonlinearFactorGraph graph;
    graph.push_back(smootherSummarization_);
    graph.push_back(smootherShortcut_);
    Values values;
    values.insert(smootherValues_);
    values.insert(theta_);
    // Calculate the summarized factor on just the new separator
    separatorSummarization = internal::calculateMarginalFactors(graph, values, newSeparatorKeys, parameters_.getEliminationFunction());
  }

  // Remove the previous marginal factors and insert the new marginal factors
  removeFactors(separatorSummarizationSlots_);
  separatorSummarizationSlots_ = insertFactors(separatorSummarization);

  if(debug) { PrintNonlinearFactorGraph(factors_, separatorSummarizationSlots_, "ConcurrentBatchFilter::synchronize  ", "New Separator Summarization:", DefaultKeyFormatter); }

  // Update the separatorValues object (should only contain the new separator keys)
  separatorValues_.clear();
  for(Key key: separatorSummarization.keys()) {
    separatorValues_.insert(key, theta_.at(key));
  }

  // Remove the marginalized factors and add them to the smoother cache
  smootherFactors_.push_back(removedFactors);
  removeFactors(removedFactorSlots);

  // Add the linearization point of the moved variables to the smoother cache
  for(Key key: keysToMove) {
    smootherValues_.insert(key, theta_.at(key));
  }

  // Remove marginalized keys from values (and separator)
  for(Key key: keysToMove) {
    theta_.erase(key);
    ordering_.erase(std::find(ordering_.begin(), ordering_.end(), key));
    delta_.erase(key);
  }

  if(debug) std::cout << "ConcurrentBatchFilter::moveSeparator  End" << std::endl;
}

/* ************************************************************************* */
}/// namespace gtsam
