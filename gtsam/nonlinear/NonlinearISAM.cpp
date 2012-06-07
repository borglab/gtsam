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

#include <gtsam/nonlinear/NonlinearISAM.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/nonlinear/Ordering.h>

#include <boost/foreach.hpp>

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
void NonlinearISAM::saveGraph(const std::string& s) const {
  isam_.saveGraph(s);
}

/* ************************************************************************* */
void NonlinearISAM::update(const NonlinearFactorGraph& newFactors,
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
    BOOST_FOREACH(const NonlinearFactorGraph::sharedFactor& factor, newFactors)
      BOOST_FOREACH(Key key, factor->keys())
        ordering_.tryInsert(key, ordering_.nVars()); // will do nothing if already present

    boost::shared_ptr<GaussianFactorGraph> linearizedNewFactors(
        newFactors.linearize(linPoint_, ordering_)->dynamicCastFactors<GaussianFactorGraph>());

    // Update ISAM
    isam_.update(*linearizedNewFactors);
  }
}

/* ************************************************************************* */
void NonlinearISAM::reorder_relinearize() {

//  std::cout << "Reordering, relinearizing..." << std::endl;

  if(factors_.size() > 0) {
    // Obtain the new linearization point
    const Values newLinPoint = estimate();

    isam_.clear();

    // Compute an ordering
    ordering_ = *factors_.orderingCOLAMD(newLinPoint);

    // Create a linear factor graph at the new linearization point
    boost::shared_ptr<GaussianFactorGraph> gfg(
        factors_.linearize(newLinPoint, ordering_)->dynamicCastFactors<GaussianFactorGraph>());

    // Just recreate the whole BayesTree
    isam_.update(*gfg);

    // Update linearization point
    linPoint_ = newLinPoint;
  }
}

/* ************************************************************************* */
Values NonlinearISAM::estimate() const {
  if(isam_.size() > 0)
    return linPoint_.retract(optimize(isam_), ordering_);
  else
    return linPoint_;
}

/* ************************************************************************* */
Matrix NonlinearISAM::marginalCovariance(Key key) const {
	return isam_.marginalCovariance(ordering_[key]);
}

/* ************************************************************************* */
void NonlinearISAM::print(const std::string& s) const {
	std::cout << "ISAM - " << s << ":" << std::endl;
	std::cout << "  ReorderInterval: " << reorderInterval_ << " Current Count: " << reorderCounter_ << std::endl;
	isam_.print("GaussianISAM");
	linPoint_.print("Linearization Point");
	ordering_.print("System Ordering");
	factors_.print("Nonlinear Graph");
}

/* ************************************************************************* */
void NonlinearISAM::printStats() const {
  gtsam::GaussianISAM::CliqueData data = isam_.getCliqueData();
  gtsam::GaussianISAM::CliqueStats stats = data.getStats();
  std::cout << "\navg Conditional Size: " << stats.avgConditionalSize;
  std::cout << "\nmax Conditional Size: " << stats.maxConditionalSize;
  std::cout << "\navg Separator Size: " << stats.avgSeparatorSize;
  std::cout << "\nmax Separator Size: " << stats.maxSeparatorSize;
  std::cout << std::endl;
}

/* ************************************************************************* */

}///\ namespace gtsam
