/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridNonlinearISAM.cpp
 * @date Sep 12, 2022
 * @author Varun Agrawal
 */

#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/inference/Ordering.h>

#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void HybridNonlinearISAM::saveGraph(const string& s,
                                    const KeyFormatter& keyFormatter) const {
  isam_.saveGraph(s, keyFormatter);
}

/* ************************************************************************* */
void HybridNonlinearISAM::update(const HybridNonlinearFactorGraph& newFactors,
                                 const Values& initialValues,
                                 const std::optional<size_t>& maxNrLeaves,
                                 const std::optional<Ordering>& ordering) {
  if (newFactors.size() > 0) {
    // Reorder and relinearize every reorderInterval updates
    if (reorderInterval_ > 0 && ++reorderCounter_ >= reorderInterval_) {
      // TODO(Varun) Relinearization doesn't take into account pruning
      reorder_relinearize();
      reorderCounter_ = 0;
    }

    factors_.push_back(newFactors);

    // Linearize new factors and insert them
    // TODO: optimize for whole config?
    linPoint_.insert(initialValues);

    boost::shared_ptr<HybridGaussianFactorGraph> linearizedNewFactors =
        newFactors.linearize(linPoint_);

    // Update ISAM
    isam_.update(*linearizedNewFactors, maxNrLeaves, ordering,
                 eliminationFunction_);
  }
}

/* ************************************************************************* */
void HybridNonlinearISAM::reorder_relinearize() {
  if (factors_.size() > 0) {
    // Obtain the new linearization point
    const Values newLinPoint = estimate();

    isam_.clear();

    // Just recreate the whole BayesTree
    // TODO: allow for constrained ordering here
    // TODO: decouple relinearization and reordering to avoid
    isam_.update(*factors_.linearize(newLinPoint), {}, {},
                 eliminationFunction_);

    // Update linearization point
    linPoint_ = newLinPoint;
  }
}

/* ************************************************************************* */
Values HybridNonlinearISAM::estimate() {
  Values result;
  if (isam_.size() > 0) {
    HybridValues values = isam_.optimize();
    assignment_ = values.discrete();
    return linPoint_.retract(values.continuous());
  } else {
    return linPoint_;
  }
}

// /* *************************************************************************
// */ Matrix HybridNonlinearISAM::marginalCovariance(Key key) const {
//   return isam_.marginalCovariance(key);
// }

/* ************************************************************************* */
void HybridNonlinearISAM::print(const string& s,
                                const KeyFormatter& keyFormatter) const {
  cout << s << "ReorderInterval: " << reorderInterval_
       << " Current Count: " << reorderCounter_ << endl;
  std::cout << "HybridGaussianISAM:" << std::endl;
  isam_.print("", keyFormatter);
  linPoint_.print("Linearization Point:\n", keyFormatter);
  std::cout << "Nonlinear Graph:" << std::endl;
  factors_.print("", keyFormatter);
}

/* ************************************************************************* */
void HybridNonlinearISAM::printStats() const {
  isam_.getCliqueData().getStats().print();
}

/* ************************************************************************* */

}  // namespace gtsam
