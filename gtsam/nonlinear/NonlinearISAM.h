/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * NonlinearISAM.h
 *
 *  Created on: Jan 19, 2010
 *      Author: Viorela Ila and Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianISAM.h>

namespace gtsam {
/**
 * Wrapper class to manage ISAM in a nonlinear context
 */
template<class Values>
class NonlinearISAM {
public:

	typedef gtsam::NonlinearFactorGraph<Values> Factors;

protected:

	/** The internal iSAM object */
	gtsam::GaussianISAM isam_;

	/** The current linearization point */
	Values linPoint_;

	/** The ordering */
	gtsam::Ordering ordering_;

	/** The original factors, used when relinearizing */
	Factors factors_;

	/** The reordering interval and counter */
	int reorderInterval_;
	int reorderCounter_;

public:

	/**
	 * Periodically reorder and relinearize
	 * @param reorderInterval is the number of updates between reorderings,
	 * 	0 never reorders (and is dangerous for memory consumption)
	 *  1 (default) reorders every time, in worse case is batch every update
	 *  typical values are 50 or 100
	 */
	NonlinearISAM(int reorderInterval = 1) : reorderInterval_(reorderInterval), reorderCounter_(0) {}

	/** Add new factors along with their initial linearization points */
	void update(const Factors& newFactors, const Values& initialValues);

	/** Return the current solution estimate */
	Values estimate();

	/** Relinearization and reordering of variables */
	void reorder_relinearize();

	/** manually add a key to the end of the ordering */
	void addKey(const Symbol& key) { ordering_.push_back(key); }

	/** replace the current ordering */
	void setOrdering(const Ordering& new_ordering) { ordering_ = new_ordering; }

	/** find the marginal covariance for a single variable */
	Matrix marginalCovariance(const Symbol& key);

	// access

	/** access the underlying bayes tree */
	const GaussianISAM& bayesTree() const { return isam_; }

	/** Return the current linearization point */
	const Values& getLinearizationPoint() const { return linPoint_; }

	/** Get the ordering */
	const gtsam::Ordering& getOrdering() const { return ordering_; }

	/** get underlying nonlinear graph */
	const Factors& getFactorsUnsafe() const { return factors_; }

	/** get counters */
	int reorderInterval() const { return reorderInterval_; }
	int reorderCounter() const { return reorderCounter_; }
};

} // \namespace gtsam
