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
size_t DeltaImpl::UpdateGaussNewtonDelta(const ISAM2::Roots& roots,
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
    Vector RSgProd = clique->conditional()->R() * gR +
                     clique->conditional()->S() * gS;

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
size_t DeltaImpl::UpdateRgProd(const ISAM2::Roots& roots,
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
VectorValues DeltaImpl::ComputeGradientSearch(const VectorValues& gradAtZero,
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
