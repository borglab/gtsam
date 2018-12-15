/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-impl.cpp
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess
 * @author  Richard Roberts
 */

#include <gtsam/base/debug.h>
#include <gtsam/config.h>            // for GTSAM_USE_TBB
#include <gtsam/inference/Symbol.h>  // for selective linearization thresholds
#include <gtsam/nonlinear/ISAM2-impl.h>

#include <boost/range/adaptors.hpp>
#include <functional>
#include <limits>
#include <string>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void ISAM2::Impl::AddFactorsStep1(const NonlinearFactorGraph& newFactors,
                                  bool useUnusedSlots,
                                  NonlinearFactorGraph* nonlinearFactors,
                                  FactorIndices* newFactorIndices) {
  newFactorIndices->resize(newFactors.size());

  if (useUnusedSlots) {
    size_t globalFactorIndex = 0;
    for (size_t newFactorIndex = 0; newFactorIndex < newFactors.size();
         ++newFactorIndex) {
      // Loop to find the next available factor slot
      do {
        // If we need to add more factors than we have room for, resize
        // nonlinearFactors, filling the new slots with NULL factors. Otherwise,
        // check if the current factor in nonlinearFactors is already used, and
        // if so, increase globalFactorIndex.  If the current factor in
        // nonlinearFactors is unused, break out of the loop and use the current
        // slot.
        if (globalFactorIndex >= nonlinearFactors->size())
          nonlinearFactors->resize(nonlinearFactors->size() +
                                   newFactors.size() - newFactorIndex);
        else if ((*nonlinearFactors)[globalFactorIndex])
          ++globalFactorIndex;
        else
          break;
      } while (true);

      // Use the current slot, updating nonlinearFactors and newFactorSlots.
      (*nonlinearFactors)[globalFactorIndex] = newFactors[newFactorIndex];
      (*newFactorIndices)[newFactorIndex] = globalFactorIndex;
    }
  } else {
    // We're not looking for unused slots, so just add the factors at the end.
    for (size_t i = 0; i < newFactors.size(); ++i)
      (*newFactorIndices)[i] = i + nonlinearFactors->size();
    nonlinearFactors->push_back(newFactors);
  }
}

/* ************************************************************************* */
KeySet ISAM2::Impl::CheckRelinearizationFull(
    const VectorValues& delta,
    const ISAM2Params::RelinearizationThreshold& relinearizeThreshold) {
  KeySet relinKeys;

  if (const double* threshold = boost::get<double>(&relinearizeThreshold)) {
    for (const VectorValues::KeyValuePair& key_delta : delta) {
      double maxDelta = key_delta.second.lpNorm<Eigen::Infinity>();
      if (maxDelta >= *threshold) relinKeys.insert(key_delta.first);
    }
  } else if (const FastMap<char, Vector>* thresholds =
                 boost::get<FastMap<char, Vector> >(&relinearizeThreshold)) {
    for (const VectorValues::KeyValuePair& key_delta : delta) {
      const Vector& threshold =
          thresholds->find(Symbol(key_delta.first).chr())->second;
      if (threshold.rows() != key_delta.second.rows())
        throw std::invalid_argument(
            "Relinearization threshold vector dimensionality for '" +
            std::string(1, Symbol(key_delta.first).chr()) +
            "' passed into iSAM2 parameters does not match actual variable "
            "dimensionality.");
      if ((key_delta.second.array().abs() > threshold.array()).any())
        relinKeys.insert(key_delta.first);
    }
  }

  return relinKeys;
}

/* ************************************************************************* */
static void CheckRelinearizationRecursiveDouble(
    double threshold, const VectorValues& delta,
    const ISAM2::sharedClique& clique, KeySet* relinKeys) {
  // Check the current clique for relinearization
  bool relinearize = false;
  for (Key var : *clique->conditional()) {
    double maxDelta = delta[var].lpNorm<Eigen::Infinity>();
    if (maxDelta >= threshold) {
      relinKeys->insert(var);
      relinearize = true;
    }
  }

  // If this node was relinearized, also check its children
  if (relinearize) {
    for (const ISAM2::sharedClique& child : clique->children) {
      CheckRelinearizationRecursiveDouble(threshold, delta, child, relinKeys);
    }
  }
}

/* ************************************************************************* */
static void CheckRelinearizationRecursiveMap(
    const FastMap<char, Vector>& thresholds, const VectorValues& delta,
    const ISAM2::sharedClique& clique, KeySet* relinKeys) {
  // Check the current clique for relinearization
  bool relinearize = false;
  for (Key var : *clique->conditional()) {
    // Find the threshold for this variable type
    const Vector& threshold = thresholds.find(Symbol(var).chr())->second;

    const Vector& deltaVar = delta[var];

    // Verify the threshold vector matches the actual variable size
    if (threshold.rows() != deltaVar.rows())
      throw std::invalid_argument(
          "Relinearization threshold vector dimensionality for '" +
          std::string(1, Symbol(var).chr()) +
          "' passed into iSAM2 parameters does not match actual variable "
          "dimensionality.");

    // Check for relinearization
    if ((deltaVar.array().abs() > threshold.array()).any()) {
      relinKeys->insert(var);
      relinearize = true;
    }
  }

  // If this node was relinearized, also check its children
  if (relinearize) {
    for (const ISAM2::sharedClique& child : clique->children) {
      CheckRelinearizationRecursiveMap(thresholds, delta, child, relinKeys);
    }
  }
}

/* ************************************************************************* */
KeySet ISAM2::Impl::CheckRelinearizationPartial(
    const ISAM2::Roots& roots, const VectorValues& delta,
    const ISAM2Params::RelinearizationThreshold& relinearizeThreshold) {
  KeySet relinKeys;
  for (const ISAM2::sharedClique& root : roots) {
    if (relinearizeThreshold.type() == typeid(double))
      CheckRelinearizationRecursiveDouble(
          boost::get<double>(relinearizeThreshold), delta, root, &relinKeys);
    else if (relinearizeThreshold.type() == typeid(FastMap<char, Vector>))
      CheckRelinearizationRecursiveMap(
          boost::get<FastMap<char, Vector> >(relinearizeThreshold), delta, root,
          &relinKeys);
  }
  return relinKeys;
}

/* ************************************************************************* */
namespace internal {
inline static void optimizeInPlace(const ISAM2::sharedClique& clique,
                                   VectorValues* result) {
  // parents are assumed to already be solved and available in result
  result->update(clique->conditional()->solve(*result));

  // starting from the root, call optimize on each conditional
  for (const ISAM2::sharedClique& child : clique->children)
    optimizeInPlace(child, result);
}
}  // namespace internal

/* ************************************************************************* */
size_t ISAM2::Impl::UpdateGaussNewtonDelta(const ISAM2::Roots& roots,
                                           const KeySet& replacedKeys,
                                           double wildfireThreshold,
                                           VectorValues* delta) {
  size_t lastBacksubVariableCount;

  if (wildfireThreshold <= 0.0) {
    // Threshold is zero or less, so do a full recalculation
    for (const ISAM2::sharedClique& root : roots)
      internal::optimizeInPlace(root, delta);
    lastBacksubVariableCount = delta->size();

  } else {
    // Optimize with wildfire
    lastBacksubVariableCount = 0;
    for (const ISAM2::sharedClique& root : roots)
      lastBacksubVariableCount += optimizeWildfireNonRecursive(
          root, wildfireThreshold, replacedKeys, delta);  // modifies delta

#if !defined(NDEBUG) && defined(GTSAM_EXTRA_CONSISTENCY_CHECKS)
    for (VectorValues::const_iterator key_delta = delta->begin();
         key_delta != delta->end(); ++key_delta) {
      assert((*delta)[key_delta->first].allFinite());
    }
#endif
  }

  return lastBacksubVariableCount;
}

/* ************************************************************************* */
namespace internal {
void updateRgProd(const ISAM2::sharedClique& clique, const KeySet& replacedKeys,
                  const VectorValues& grad, VectorValues* RgProd,
                  size_t* varsUpdated) {
  // Check if any frontal or separator keys were recalculated, if so, we need
  // update deltas and recurse to children, but if not, we do not need to
  // recurse further because of the running separator property.
  bool anyReplaced = false;
  for (Key j : *clique->conditional()) {
    if (replacedKeys.exists(j)) {
      anyReplaced = true;
      break;
    }
  }

  if (anyReplaced) {
    // Update the current variable
    // Get VectorValues slice corresponding to current variables
    Vector gR =
        grad.vector(KeyVector(clique->conditional()->beginFrontals(),
                                    clique->conditional()->endFrontals()));
    Vector gS =
        grad.vector(KeyVector(clique->conditional()->beginParents(),
                                    clique->conditional()->endParents()));

    // Compute R*g and S*g for this clique
    Vector RSgProd = clique->conditional()->get_R() * gR +
                     clique->conditional()->get_S() * gS;

    // Write into RgProd vector
    DenseIndex vectorPosition = 0;
    for (Key frontal : clique->conditional()->frontals()) {
      Vector& RgProdValue = (*RgProd)[frontal];
      RgProdValue = RSgProd.segment(vectorPosition, RgProdValue.size());
      vectorPosition += RgProdValue.size();
    }

    // Now solve the part of the Newton's method point for this clique
    // (back-substitution)
    // (*clique)->solveInPlace(deltaNewton);

    *varsUpdated += clique->conditional()->nrFrontals();

    // Recurse to children
    for (const ISAM2::sharedClique& child : clique->children) {
      updateRgProd(child, replacedKeys, grad, RgProd, varsUpdated);
    }
  }
}
}  // namespace internal

/* ************************************************************************* */
size_t ISAM2::Impl::UpdateRgProd(const ISAM2::Roots& roots,
                                 const KeySet& replacedKeys,
                                 const VectorValues& gradAtZero,
                                 VectorValues* RgProd) {
  // Update variables
  size_t varsUpdated = 0;
  for (const ISAM2::sharedClique& root : roots) {
    internal::updateRgProd(root, replacedKeys, gradAtZero, RgProd,
                           &varsUpdated);
  }

  return varsUpdated;
}

/* ************************************************************************* */
VectorValues ISAM2::Impl::ComputeGradientSearch(const VectorValues& gradAtZero,
                                                const VectorValues& RgProd) {
  // Compute gradient squared-magnitude
  const double gradientSqNorm = gradAtZero.dot(gradAtZero);

  // Compute minimizing step size
  double RgNormSq = RgProd.vector().squaredNorm();
  double step = -gradientSqNorm / RgNormSq;

  // Compute steepest descent point
  return step * gradAtZero;
}

}  // namespace gtsam
