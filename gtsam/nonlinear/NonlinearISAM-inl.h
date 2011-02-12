/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * NonlinearISAM-inl.h
 *
 *  Created on: Jan 19, 2010
 *      Author: Viorela Ila and Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <boost/foreach.hpp>

using namespace gtsam;

/* ************************************************************************* */
template<class Values>
void NonlinearISAM<Values>::update(const Factors& newFactors, const Values& initialValues) {

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
    // FIXME: should just loop over new values
    BOOST_FOREACH(const typename Factors::sharedFactor& factor, newFactors)
      BOOST_FOREACH(const Symbol& key, factor->keys())
        ordering_.tryInsert(key, ordering_.nVars()); // will do nothing if already present

    boost::shared_ptr<GaussianFactorGraph> linearizedNewFactors(
        newFactors.linearize(linPoint_, ordering_)->template dynamicCastFactors<GaussianFactorGraph>());

    // Update ISAM
    isam_.update(*linearizedNewFactors);
  }
}

/* ************************************************************************* */
template<class Values>
void NonlinearISAM<Values>::reorder_relinearize() {

//  cout << "Reordering, relinearizing..." << endl;

  // Obtain the new linearization point
  const Values newLinPoint = estimate();

  isam_.clear();

  // Compute an ordering
  ordering_ = *factors_.orderingCOLAMD(newLinPoint);

  // Create a linear factor graph at the new linearization point
  boost::shared_ptr<GaussianFactorGraph> gfg(
      factors_.linearize(newLinPoint, ordering_)->template dynamicCastFactors<GaussianFactorGraph>());

  // Just recreate the whole BayesTree
  isam_.update(*gfg);

  // Update linearization point
  linPoint_ = newLinPoint;
}

/* ************************************************************************* */
template<class Values>
Values NonlinearISAM<Values>::estimate() const {
  if(isam_.size() > 0)
    return linPoint_.expmap(optimize(isam_), ordering_);
  else
    return linPoint_;
}

/* ************************************************************************* */
template<class Values>
Matrix NonlinearISAM<Values>::marginalCovariance(const Symbol& key) const {
	Matrix covariance; Vector mean;
	boost::tie(mean, covariance) = isam_.marginal(ordering_[key]);
	return covariance;
}
