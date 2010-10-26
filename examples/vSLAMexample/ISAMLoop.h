/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * ISAMLoop.h
 *
 *  Created on: Jan 19, 2010
 *      Author: Viorela Ila and Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianISAM.h>


template<class Values>
class ISAMLoop {
public:

  typedef gtsam::NonlinearFactorGraph<Values> Factors;

public:
//protected:

  /** The internal iSAM object */
  gtsam::GaussianISAM isam;

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
  ISAMLoop() : reorderInterval_(0), reorderCounter_(0) {}

  /** Periodically reorder and relinearize */
  ISAMLoop(int reorderInterval) : reorderInterval_(reorderInterval), reorderCounter_(0) {}

  /** Add new factors along with their initial linearization points */
  void update(const Factors& newFactors, const Values& initialValues);

  /** Return the current solution estimate */
  Values estimate();

  /** Return the current linearization point */
  const Values& getLinearizationPoint() { return linPoint_; }

  /** Get the ordering */
  const gtsam::Ordering& getOrdering() const { return ordering_; }

  const Factors& getFactorsUnsafe() { return factors_; }

  /**
   * Relinearization and reordering of variables
   */
  void reorder_relinearize();

};
