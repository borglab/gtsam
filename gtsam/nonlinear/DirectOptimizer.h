/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DirectOptimizer.h
 * @brief 
 * @author Richard Roberts
 * @date Apr 1, 2012
 */

#pragma once

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

class DirectOptimizerParams : public NonlinearOptimizerParams {
public:
  /** See DoglegParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See DoglegParams::factorization */
  enum Factorization {
    LDL,
    QR,
  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: LDL)

  DirectOptimizerParams() :
    elimination(MULTIFRONTAL), factorization(LDL) {}

  virtual ~DirectOptimizerParams() {}

  virtual void print(const std::string& str = "") const {
    NonlinearOptimizerParams::print(str);
    if(elimination == MULTIFRONTAL)
      std::cout << "         elimination method: MULTIFRONTAL\n";
    else if(elimination == SEQUENTIAL)
      std::cout << "         elimination method: SEQUENTIAL\n";
    else
      std::cout << "         elimination method: (invalid)\n";

    if(factorization == LDL)
      std::cout << "       factorization method: LDL\n";
    else if(factorization == QR)
      std::cout << "       factorization method: QR\n";
    else
      std::cout << "       factorization method: (invalid)\n";

    std::cout.flush();
  }
};

/**
 * This is an intermediate base class for NonlinearOptimizers that use direct
 * solving, i.e. factorization.  This class is here to reduce code duplication
 * for storing the ordering, having factorization and elimination parameters,
 * etc.
 */
class DirectOptimizer : public NonlinearOptimizer {
public:
  typedef boost::shared_ptr<const Ordering> SharedOrdering;
  typedef boost::shared_ptr<NonlinearOptimizer> shared_ptr;

  virtual NonlinearOptimizer::shared_ptr update(const SharedGraph& newGraph) const;

  virtual shared_ptr update(const SharedOrdering& newOrdering) const;

  virtual shared_ptr update(const SharedGraph& newGraph, const SharedOrdering& newOrdering) const;

  /** Access the variable ordering used by this optimizer */
  const SharedOrdering& ordering() const { return ordering_; }

protected:

  const bool colamdOrdering_;
  const SharedOrdering ordering_;

  DirectOptimizer(const SharedGraph& graph,
      const SharedOrdering ordering = SharedOrdering()) :
        NonlinearOptimizer(graph),
        colamdOrdering_(!ordering || ordering->size() == 0),
        ordering_(colamdOrdering_ ? graph_->orderingCOLAMD(*values_) : ordering) {}
};

} /* namespace gtsam */
