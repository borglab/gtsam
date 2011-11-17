/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearISAM-inl.h
 * @date Jan 19, 2010
 * @author Viorela Ila and Richard Roberts
 */

#pragma once

#include <iostream>

#include <boost/foreach.hpp>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/nonlinear/Ordering.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
template<class VALUES, class GRAPH>
void NonlinearISAM<VALUES,GRAPH>::update(const Factors& newFactors,
		const Values& initialValues) {

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
template<class VALUES, class GRAPH>
void NonlinearISAM<VALUES,GRAPH>::reorder_relinearize() {

//  cout << "Reordering, relinearizing..." << endl;

  if(factors_.size() > 0) {
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
}

/* ************************************************************************* */
template<class VALUES, class GRAPH>
VALUES NonlinearISAM<VALUES,GRAPH>::estimate() const {
  if(isam_.size() > 0)
    return linPoint_.retract(optimize(isam_), ordering_);
  else
    return linPoint_;
}

/* ************************************************************************* */
template<class VALUES, class GRAPH>
Matrix NonlinearISAM<VALUES,GRAPH>::marginalCovariance(const Symbol& key) const {
	return isam_.marginalCovariance(ordering_[key]);
}

/* ************************************************************************* */
template<class VALUES, class GRAPH>
void NonlinearISAM<VALUES,GRAPH>::print(const std::string& s) const {
	cout << "ISAM - " << s << ":" << endl;
	cout << "  ReorderInterval: " << reorderInterval_ << " Current Count: " << reorderCounter_ << endl;
	isam_.print("GaussianISAM");
	linPoint_.print("Linearization Point");
	ordering_.print("System Ordering");
	factors_.print("Nonlinear Graph");
}
