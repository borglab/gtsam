/*
 * ISAMLoop.cpp
 *
 *  Created on: Jan 19, 2010
 *      Author: Viorela Ila and Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/nonlinear/Ordering.h>
#include <boost/foreach.hpp>

#include "ISAMLoop.h"

using namespace gtsam;

/* ************************************************************************* */
template<class Values>
void ISAMLoop<Values>::update(const Factors& newFactors, const Values& initialValues) {

  if(newFactors.size() > 0) {

      // Reorder and relinearize every reorderInterval updates
      if(reorderInterval_ > 0 && ++reorderCounter_ >= reorderInterval_) {
        reorder_relinearize();
        reorderCounter_ = 0;
      }

      factors_.push_back(newFactors);

      // Linearize new factors and insert them
      // TODO: optimize for whole config?
      linPoint_.insert(initialValues);

      // Augment ordering
      BOOST_FOREACH(const typename Factors::sharedFactor& factor, newFactors) {
          BOOST_FOREACH(const Symbol& key, factor->keys()) {
            ordering_.tryInsert(key, ordering_.nVars());
          }
      }

      boost::shared_ptr<GaussianFactorGraph> linearizedNewFactors(newFactors.linearize(linPoint_, ordering_));

      // Update ISAM
      isam.update(*linearizedNewFactors);
  }
}

/* ************************************************************************* */
template<class Values>
void ISAMLoop<Values>::reorder_relinearize() {

  cout << "Reordering, relinearizing..." << endl;

  // Obtain the new linearization point
  const Values newLinPoint = estimate();

  isam.clear();

  // Compute an ordering
  ordering_ = *factors_.orderingCOLAMD(newLinPoint);

  // Create a linear factor graph at the new linearization point
  boost::shared_ptr<GaussianFactorGraph> gfg(factors_.linearize(newLinPoint, ordering_));

  // Just recreate the whole BayesTree
  isam.update(*gfg);

  // Update linearization point
  linPoint_ = newLinPoint;
}

/* ************************************************************************* */
template<class Values>
Values ISAMLoop<Values>::estimate() {
  if(isam.size() > 0)
    return linPoint_.expmap(optimize(isam), ordering_);
  else
    return linPoint_;
}
