/**
 * @file IterativeOptimizationParameters.h
 * @date Oct 22, 2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

namespace gtsam {

// a container for all related parameters
struct IterativeOptimizationParameters {

public:

  typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;

  typedef enum {
    SILENT = 0,
    ERROR,
  } verbosityLevel;

public:
  size_t minIterations_;
  size_t maxIterations_;
  size_t reset_; // number of iterations before reset, for cg and gmres
  double epsilon_rel_; // relative error
  double epsilon_abs_; // absolute error
  verbosityLevel verbosity_;
  bool est_cond_ ;
  std::map<std::string, std::string> sandbox_;

public:
  IterativeOptimizationParameters()
  : minIterations_(1), maxIterations_(500), reset_(501),
    epsilon_rel_(1e-3), epsilon_abs_(1e-3), verbosity_(SILENT), est_cond_(false) {}

  IterativeOptimizationParameters(
      const IterativeOptimizationParameters &p) :
    minIterations_(p.minIterations_), maxIterations_(p.maxIterations_), reset_(p.reset_),
    epsilon_rel_(p.epsilon_rel_), epsilon_abs_(p.epsilon_abs_), verbosity_(p.verbosity_),
    est_cond_(p.est_cond_){ }

  IterativeOptimizationParameters(size_t minIterations, size_t maxIterations, size_t reset,
    double epsilon, double epsilon_abs, verbosityLevel verbosity = ERROR, bool est_cond = false) :
    minIterations_(minIterations), maxIterations_(maxIterations), reset_(reset),
    epsilon_rel_(epsilon), epsilon_abs_(epsilon_abs), verbosity_(verbosity), est_cond_(est_cond) {}

  size_t minIterations() const { return minIterations_; }
  size_t maxIterations() const { return maxIterations_; }
  size_t reset() const { return reset_; }
  double epsilon() const { return epsilon_rel_; }
  double epsilon_rel() const { return epsilon_rel_; }
  double epsilon_abs() const { return epsilon_abs_; }
  verbosityLevel verbosity() const { return verbosity_; }
  bool est_cond() const { return est_cond_ ; }
};

}
