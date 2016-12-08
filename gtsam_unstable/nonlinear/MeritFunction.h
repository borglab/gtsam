/*
 * SQPLineSearch2.h
 * @brief:
 * @date: December 7, 2016
 * @author: Duy-Nguyen Ta
 * @author: Ivan Dario Jimenez
 */


/**
 * Merit function goes with Betts' line search SQP implementation
 * Betts10book 2.27
 */

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/nonlinear/NP.h>

namespace gtsam {
class MeritFunction {
  const NP & program_;
  const GaussianFactorGraph::shared_ptr linearizedCost_;
  const GaussianFactorGraph::shared_ptr lagrangianGraph_;
  Values x_;
  VectorValues p_, gradf_;

public:
  
  /// Constructor
  MeritFunction(const NP & program,
                const GaussianFactorGraph::shared_ptr linearizedCost,
                const GaussianFactorGraph::shared_ptr lagrangianGraph, const Values& x,
                const VectorValues& p);
  
  /// Update predicted solution, Betts10book, 2.30
  boost::tuple<Values, VectorValues, VectorValues> update(double alpha) const;
  
  /// Compute 1-norm of the constraints ||c(x)||_1
  double constraintNorm1(const Values x) const;
  
  /// phi(alpha,mu)
  double phi(double alpha, double mu) const;
  
  /// Dk(mu)
  double D(double mu) const;
  
  /// Nocedal06book, 18.36
  double computeNewMu(double currentMu) const;
  
  double ptHp(const GaussianFactorGraph::shared_ptr linear, const VectorValues& p) const;
  
};
}