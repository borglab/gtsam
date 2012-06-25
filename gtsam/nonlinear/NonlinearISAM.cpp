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

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void NonlinearISAM::saveGraph(const string& s, const KeyFormatter& keyFormatter) const {

  // Create an index formatter that looks up the Key in an inverse ordering, then
  // formats the key using the provided key formatter.
  struct OrderingIndexFormatter {
    Ordering::InvertedMap inverseOrdering;
    const KeyFormatter& keyFormatter;
    OrderingIndexFormatter(const Ordering& ordering, const KeyFormatter& keyFormatter) :
        inverseOrdering(ordering.invert()), keyFormatter(keyFormatter) {}
    string operator()(Index index) {
      return keyFormatter(inverseOrdering.at(index));
    }
  };

  isam_.saveGraph(s, OrderingIndexFormatter(ordering_, keyFormatter));
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
    // TODO: allow for ordering constraints within the new variables
    // FIXME: should just loop over new values
    BOOST_FOREACH(const NonlinearFactorGraph::sharedFactor& factor, newFactors)
      BOOST_FOREACH(Key key, factor->keys())
        ordering_.tryInsert(key, ordering_.nVars()); // will do nothing if already present

    boost::shared_ptr<GaussianFactorGraph> linearizedNewFactors = newFactors.linearize(linPoint_, ordering_);

    // Update ISAM
    isam_.update(*linearizedNewFactors);
  }
}

/* ************************************************************************* */
void NonlinearISAM::reorder_relinearize() {

//  cout << "Reordering, relinearizing..." << endl;

  if(factors_.size() > 0) {
    // Obtain the new linearization point
    const Values newLinPoint = estimate();

    isam_.clear();

    // Compute an ordering
    // TODO: allow for constrained ordering here
    ordering_ = *factors_.orderingCOLAMD(newLinPoint);

    // Create a linear factor graph at the new linearization point
    // TODO: decouple relinearization and reordering to avoid
    boost::shared_ptr<GaussianFactorGraph> gfg = factors_.linearize(newLinPoint, ordering_);

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
void NonlinearISAM::print(const string& s, const KeyFormatter& keyFormatter) const {
	cout << s << "ReorderInterval: " << reorderInterval_ << " Current Count: " << reorderCounter_ << endl;
	isam_.print("GaussianISAM:\n");
	linPoint_.print("Linearization Point:\n", keyFormatter);
	ordering_.print("System Ordering:\n", keyFormatter);
	factors_.print("Nonlinear Graph:\n", keyFormatter);
}

/* ************************************************************************* */
void NonlinearISAM::printStats() const {
  gtsam::GaussianISAM::CliqueData data = isam_.getCliqueData();
  gtsam::GaussianISAM::CliqueStats stats = data.getStats();
  cout << "\navg Conditional Size: " << stats.avgConditionalSize;
  cout << "\nmax Conditional Size: " << stats.maxConditionalSize;
  cout << "\navg Separator Size: " << stats.avgSeparatorSize;
  cout << "\nmax Separator Size: " << stats.maxSeparatorSize;
  cout << endl;
}

/* ************************************************************************* */

}///\ namespace gtsam
