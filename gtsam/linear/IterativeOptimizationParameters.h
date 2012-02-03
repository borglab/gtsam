/**
 * @file IterativeOptimizationParameters.h
 * @date Oct 22, 2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

struct DimSpec;

// a container for all related parameters
struct IterativeOptimizationParameters {

public:

  typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;

  typedef enum {
    SILENT = 0,
    ERROR,
  } verbosityLevel;

public:
  int maxIterations_;
  int reset_; // number of iterations before reset, for cg and gmres
  double epsilon_; // relative error
  double epsilon_abs_; // absolute error
  verbosityLevel verbosity_;
  size_t nReduce_ ;
  boost::shared_ptr<DimSpec> skeleton_spec_;
  bool est_cond_ ;

public:
  IterativeOptimizationParameters() :
    maxIterations_(500), reset_(501), epsilon_(1e-3), epsilon_abs_(1e-3),
        verbosity_(SILENT), nReduce_(0), skeleton_spec_(), est_cond_(false) {
  }

  IterativeOptimizationParameters(
      const IterativeOptimizationParameters &parameters) :
    maxIterations_(parameters.maxIterations_), reset_(parameters.reset_),
        epsilon_(parameters.epsilon_), epsilon_abs_(parameters.epsilon_abs_),
        verbosity_(parameters.verbosity_),
        nReduce_(parameters.nReduce_),
        skeleton_spec_(parameters.skeleton_spec_),
        est_cond_(parameters.est_cond_){
  }

  IterativeOptimizationParameters(int maxIterations, double epsilon,
      double epsilon_abs, verbosityLevel verbosity = ERROR, int reset = -1, bool est_cond=false) :
    maxIterations_(maxIterations), reset_(reset), epsilon_(epsilon),
        epsilon_abs_(epsilon_abs), verbosity_(verbosity),
        nReduce_(0),
        skeleton_spec_(),
        est_cond_(est_cond) {
    if (reset_ == -1)
      reset_ = maxIterations_ + 1;
  }

  int maxIterations() const {
    return maxIterations_;
  }
  int reset() const {
    return reset_;
  }
  double epsilon() const {
    return epsilon_;
  }
  double epsilon_abs() const {
    return epsilon_abs_;
  }
  verbosityLevel verbosity() const {
    return verbosity_;
  }
  bool est_cond() const {
    return est_cond_ ;
  }
};

struct DimSpec: public std::vector<size_t> {

  typedef std::vector<size_t> Base;
  typedef boost::shared_ptr<DimSpec> shared_ptr;

  DimSpec() :
    Base() {
  }
  DimSpec(size_t n) :
    Base(n) {
  }
  DimSpec(size_t n, size_t init) :
    Base(n, init) {
  }
  DimSpec(const VectorValues &V) :
    Base(V.size()) {
    const size_t n = V.size();
    for (size_t i = 0; i < n; ++i) {
      (*this)[i] = V[i].rows();
    }
  }
};

}
