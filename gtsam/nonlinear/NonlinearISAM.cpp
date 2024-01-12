/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearISAM.cpp
 * @date Jan 19, 2010
 * @author Viorela Ila and Richard Roberts
 */

#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>

#include <iostream>

using namespace std;

namespace gtsam {


/* ************************************************************************* */
void NonlinearISAM::saveGraph(const string& s, const KeyFormatter& keyFormatter) const {
  isam_.saveGraph(s, keyFormatter);
}

/* ************************************************************************* */
void NonlinearISAM::update(const NonlinearFactorGraph& newFactors, const Values& initialValues) {

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

    std::shared_ptr<GaussianFactorGraph> linearizedNewFactors = newFactors.linearize(linPoint_);

    // Update ISAM
    isam_.update(*linearizedNewFactors, eliminationFunction_);
  }
}

/* ************************************************************************* */
void NonlinearISAM::reorder_relinearize() {

//  cout << "Reordering, relinearizing..." << endl;

  if(factors_.size() > 0) {
    // Obtain the new linearization point
    const Values newLinPoint = estimate();

    isam_.clear();

    // Just recreate the whole BayesTree
    // TODO: allow for constrained ordering here
    // TODO: decouple relinearization and reordering to avoid
    isam_.update(*factors_.linearize(newLinPoint), eliminationFunction_);

    // Update linearization point
    linPoint_ = newLinPoint;
  }
}

/* ************************************************************************* */
Values NonlinearISAM::estimate() const {
  if(isam_.size() > 0)
    return linPoint_.retract(isam_.optimize());
  else
    return linPoint_;
}

/* ************************************************************************* */
Matrix NonlinearISAM::marginalCovariance(Key key) const {
  return isam_.marginalCovariance(key);
}

/* ************************************************************************* */
void NonlinearISAM::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "ReorderInterval: " << reorderInterval_ << " Current Count: " << reorderCounter_ << endl;
  isam_.print("GaussianISAM:\n");
  linPoint_.print("Linearization Point:\n", keyFormatter);
  factors_.print("Nonlinear Graph:\n", keyFormatter);
}

/* ************************************************************************* */
void NonlinearISAM::printStats() const {
  isam_.getCliqueData().getStats().print();
}

/* ************************************************************************* */

}///\ namespace gtsam
