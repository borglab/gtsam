/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SuccessiveLinearizationOptimizer.h
 * @brief 
 * @author Richard Roberts
 * @date Apr 1, 2012
 */

#pragma once

#include <boost/variant.hpp>

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

class SuccessiveLinearizationParams;
class SuccessiveLinearizationState;

/**
 * This is an intermediate base class for NonlinearOptimizers that use direct
 * solving, i.e. factorization.  This class is here to reduce code duplication
 * for storing the ordering, having factorization and elimination parameters,
 * etc.
 */
class SuccessiveLinearizationOptimizer : public NonlinearOptimizer {
public:

  typedef boost::shared_ptr<const SuccessiveLinearizationParams> SharedParams;
  typedef boost::shared_ptr<const SuccessiveLinearizationOptimizer> shared_ptr;

protected:

  SuccessiveLinearizationOptimizer(const SharedGraph& graph) : NonlinearOptimizer(graph) {}

  const Ordering& ordering(const Values& values) const;

private:

  mutable boost::optional<Ordering> ordering_; // Mutable because we set/compute it when needed and cache it
};

class SuccessiveLinearizationParams : public NonlinearOptimizerParams {
public:
  /** See SuccessiveLinearizationParams::elimination */
  enum Elimination {
    MULTIFRONTAL,
    SEQUENTIAL
  };

  /** See SuccessiveLinearizationParams::factorization */
  enum Factorization {
    LDL,
    QR,
  };

  Elimination elimination; ///< The elimination algorithm to use (default: MULTIFRONTAL)
  Factorization factorization; ///< The numerical factorization (default: LDL)
  boost::optional<Ordering> ordering; ///< The variable elimination ordering, or empty to use COLAMD (default: empty)

  SuccessiveLinearizationParams() :
    elimination(MULTIFRONTAL), factorization(LDL) {}

  virtual ~SuccessiveLinearizationParams() {}

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

    if(ordering)
      std::cout << "                   ordering: custom\n";
    else
      std::cout << "                   ordering: COLAMD\n";

    std::cout.flush();
  }
};

class SuccessiveLinearizationState : public NonlinearOptimizerState {
};

} /* namespace gtsam */
