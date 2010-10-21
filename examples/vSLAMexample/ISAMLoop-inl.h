/*
 * ISAMLoop.cpp
 *
 *  Created on: Jan 19, 2010
 *      Author: Viorela Ila and Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
//#include <BayesTree-inl.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/nonlinear/Ordering.h>
//#include <gtsam/inference/IndexTable.h>
#include <boost/foreach.hpp>

#include "ISAMLoop.h"

using namespace gtsam;

/* ************************************************************************* */
template<class Values>
void ISAMLoop<Values>::update(const Factors& newFactors, const Values& initialValues) {
  // Reorder and relinearize every reorderInterval updates
  if(newFactors.size() > 0) {
    if(reorderInterval_ > 0 && ++reorderCounter_ >= reorderInterval_) {
      reorder_relinearize();
      reorderCounter_ = 0;
    }

    factors_.push_back(newFactors);

//    BOOST_FOREACH(typename Factors::sharedFactor f, newFactors) {
//      f->print("Adding factor: ");
//    }

    // Linearize new factors and insert them
    // TODO: optimize for whole config?
    linPoint_.insert(initialValues);

    // Augment ordering
    BOOST_FOREACH(const typename Factors::sharedFactor& factor, newFactors) {
        BOOST_FOREACH(const Symbol& key, factor->keys()) {
          ordering_.tryInsert(key, ordering_.nVars());
        }
    }

    ordering_.print();
    newFactors.linearize(linPoint_, ordering_);
    cout << "Don linearize!" << endl;

    boost::shared_ptr<GaussianFactorGraph> linearizedNewFactors(newFactors.linearize(linPoint_, ordering_));

    cout << "After linearize: " << endl;
    BOOST_FOREACH(GaussianFactorGraph::sharedFactor f, *linearizedNewFactors) {
      f->print("Linearized factor: ");
    }
    isam.update(*linearizedNewFactors);
  }
}

/* ************************************************************************* */
template<class Values>
void ISAMLoop<Values>::reorder_relinearize() {

  //cout << "Reordering " << reorderCounter_;

  cout << "Reordering, relinearizing..." << endl;

  // Obtain the new linearization point
  const Values newLinPoint = estimate();

  isam.clear();

  // Compute an ordering
  ordering_ = *factors_.orderingCOLAMD(newLinPoint);

//  cout << "Got estimate" << endl;
//  newLinPoint.print("newLinPoint");
//  factors_.print("factors");

  // Create a linear factor graph at the new linearization point
  boost::shared_ptr<GaussianFactorGraph> gfg(factors_.linearize(newLinPoint, ordering_));

  // Just recreate the whole BayesTree
  isam.update(*gfg);

  //cout << "Reeliminating..." << endl;

//  // Eliminate linear factor graph to a BayesNet with colamd ordering
//  Ordering ordering = gfg->getOrdering();
//  const BayesNet<GaussianConditional> bn(
//      eliminate<GaussianFactor, GaussianConditional>(*gfg, ordering));
//
////  cout << "Rebuilding BayesTree..." << endl;
//
//  // Replace the BayesTree with a new one
//  isam.clear();
//  BOOST_REVERSE_FOREACH(const GaussianISAM::sharedConditional c, bn) {
//    isam.insert(c, ordering);
//  }

  linPoint_ = newLinPoint;

//  cout << "Done!" << endl;
}

/* ************************************************************************* */
template<class Values>
Values ISAMLoop<Values>::estimate() {
//  cout << "ISAMLoop::estimate(): " << endl;
//  linPoint_.print("linPoint_");
//  isam.print("isam");
  if(isam.size() > 0)
    return linPoint_.expmap(optimize(isam), ordering_);
  else
    return linPoint_;
}
