/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   IterativeSolver.h
 * @brief  Some support classes for iterative solvers
 * @date   2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/linear/KeyInfo.h>

#include <memory>

#include <iosfwd>
#include <string>
#include <map>
#include <optional>

namespace gtsam {

// Forward declarations
class GaussianFactorGraph;
class Values;
class VectorValues;

/**
 * parameters for iterative linear solvers
 */
class IterativeOptimizationParameters {
 public:
  typedef std::shared_ptr<IterativeOptimizationParameters> shared_ptr;
  enum Verbosity { SILENT = 0, COMPLEXITY, ERROR };

 protected:
  Verbosity verbosity_;

 public:

  IterativeOptimizationParameters(Verbosity v = SILENT) :
      verbosity_(v) {
  }

  virtual ~IterativeOptimizationParameters() {
  }

  /* utility */
  inline Verbosity verbosity() const {
    return verbosity_;
  }
  GTSAM_EXPORT std::string getVerbosity() const;
  GTSAM_EXPORT void setVerbosity(const std::string &s);

  /* matlab interface */
  GTSAM_EXPORT void print() const;

  /* virtual print function */
  GTSAM_EXPORT virtual void print(std::ostream &os) const;

  GTSAM_EXPORT virtual bool equals(const IterativeOptimizationParameters &other,
                                   double tol = 1e-9) const;

  /* for serialization */
  GTSAM_EXPORT friend std::ostream &operator<<(
      std::ostream &os, const IterativeOptimizationParameters &p);

  GTSAM_EXPORT static Verbosity verbosityTranslator(const std::string &s);
  GTSAM_EXPORT static std::string verbosityTranslator(Verbosity v);
};

/**
 * Base class for Iterative Solvers like SubgraphSolver
 */
class IterativeSolver {
public:
  typedef std::shared_ptr<IterativeSolver> shared_ptr;
  IterativeSolver() {
  }
  virtual ~IterativeSolver() {
  }

  /* interface to the nonlinear optimizer, without metadata, damping and initial estimate */
  GTSAM_EXPORT VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo* = nullptr,
      const std::map<Key, Vector>* lambda = nullptr);

  /* interface to the nonlinear optimizer, without initial estimate */
  GTSAM_EXPORT VectorValues optimize(const GaussianFactorGraph &gfg, const KeyInfo &keyInfo,
      const std::map<Key, Vector> &lambda);

  /* interface to the nonlinear optimizer that the subclasses have to implement */
  virtual VectorValues optimize(const GaussianFactorGraph &gfg,
      const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda,
      const VectorValues &initial) = 0;

};

} // \ namespace gtsam
