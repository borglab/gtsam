/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ISAM2-inl.h
 * @brief 
 * @author Richard Roberts
 * @date Mar 16, 2012
 */


#pragma once


#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/* ************************************************************************* */
template<class VALUE>
VALUE ISAM2::calculateEstimate(Key key) const {
  const Index index = getOrdering()[key];
  const SubVector delta = getDelta()[index];
  return theta_.at<VALUE>(key).retract(delta);
}

/* ************************************************************************* */
namespace internal {
template<class CLIQUE>
void optimizeWildfire(const boost::shared_ptr<CLIQUE>& clique, double threshold,
    std::vector<bool>& changed, const std::vector<bool>& replaced, VectorValues& delta, int& count) {
  // if none of the variables in this clique (frontal and separator!) changed
  // significantly, then by the running intersection property, none of the
  // cliques in the children need to be processed

  // Are any clique variables part of the tree that has been redone?
  bool cliqueReplaced = replaced[(*clique)->frontals().front()];
#ifndef NDEBUG
  BOOST_FOREACH(Index frontal, (*clique)->frontals()) {
    assert(cliqueReplaced == replaced[frontal]);
  }
#endif

  // If not redone, then has one of the separator variables changed significantly?
  bool recalculate = cliqueReplaced;
  if(!recalculate) {
    BOOST_FOREACH(Index parent, (*clique)->parents()) {
      if(changed[parent]) {
        recalculate = true;
        break;
      }
    }
  }

  // Solve clique if it was replaced, or if any parents were changed above the
  // threshold or themselves replaced.
  if(recalculate) {

    // Temporary copy of the original values, to check how much they change
    std::vector<Vector> originalValues((*clique)->nrFrontals());
    GaussianConditional::const_iterator it;
    for(it = (*clique)->beginFrontals(); it!=(*clique)->endFrontals(); it++) {
      originalValues[it - (*clique)->beginFrontals()] = delta[*it];
    }

    // Back-substitute
    (*clique)->solveInPlace(delta);
    count += (*clique)->nrFrontals();

    // Whether the values changed above a threshold, or always true if the
    // clique was replaced.
    bool valuesChanged = cliqueReplaced;
    for(it = (*clique)->beginFrontals(); it!=(*clique)->endFrontals(); it++) {
      if(!valuesChanged) {
        const Vector& oldValue(originalValues[it - (*clique)->beginFrontals()]);
        const SubVector& newValue(delta[*it]);
        if((oldValue - newValue).lpNorm<Eigen::Infinity>() >= threshold) {
          valuesChanged = true;
          break;
        }
      } else
        break;
    }

    // If the values were above the threshold or this clique was replaced
    if(valuesChanged) {
      // Set changed flag for each frontal variable and leave the new values
      BOOST_FOREACH(Index frontal, (*clique)->frontals()) {
        changed[frontal] = true;
      }
    } else {
      // Replace with the old values
      for(it = (*clique)->beginFrontals(); it!=(*clique)->endFrontals(); it++) {
        delta[*it] = originalValues[it - (*clique)->beginFrontals()];
      }
    }

    // Recurse to children
    BOOST_FOREACH(const typename CLIQUE::shared_ptr& child, clique->children_) {
      optimizeWildfire(child, threshold, changed, replaced, delta, count);
    }
  }
}
}

/* ************************************************************************* */
template<class CLIQUE>
int optimizeWildfire(const boost::shared_ptr<CLIQUE>& root, double threshold, const std::vector<bool>& keys, VectorValues& delta) {
  std::vector<bool> changed(keys.size(), false);
  int count = 0;
  // starting from the root, call optimize on each conditional
  if(root)
    internal::optimizeWildfire(root, threshold, changed, keys, delta, count);
  return count;
}

/* ************************************************************************* */
template<class CLIQUE>
void nnz_internal(const boost::shared_ptr<CLIQUE>& clique, int& result) {
  int dimR = (*clique)->dim();
  int dimSep = (*clique)->get_S().cols() - dimR;
  result += ((dimR+1)*dimR)/2 + dimSep*dimR;
  // traverse the children
  BOOST_FOREACH(const typename CLIQUE::shared_ptr& child, clique->children_) {
    nnz_internal(child, result);
  }
}

/* ************************************************************************* */
template<class CLIQUE>
int calculate_nnz(const boost::shared_ptr<CLIQUE>& clique) {
  int result = 0;
  // starting from the root, add up entries of frontal and conditional matrices of each conditional
  nnz_internal(clique, result);
  return result;
}

}


