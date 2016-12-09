/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearEqualityFactorGraph.h
 * @author  Ivan Dario Jimenez
 * @date    Dec 15, 2014
 */

#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>

namespace gtsam {
  
class NonlinearCostFactorGraph: public FactorGraph<NonlinearConstraint> {
public:
  
  typedef boost::shared_ptr<NonlinearCostFactorGraph> shared_ptr;
  
  NonlinearCostFactorGraph() {
  }

  GaussianFactorGraph::shared_ptr linearize(const Values& linearizationPoint) const{
    GaussianFactorGraph::shared_ptr linearizedGraph(new GaussianFactorGraph());
    
    for(const NonlinearConstraint::shared_ptr & factor: *this){
      linearizedGraph->push_back(factor->linearize(linearizationPoint));
    }
    return linearizedGraph;
  }
  
  GaussianFactorGraph::shared_ptr secondOrderApproximation(const Values& linearizationPoint) const {
    GaussianFactorGraph::shared_ptr linearizedGraph(new GaussianFactorGraph());
    for (const NonlinearConstraint::shared_ptr & factor : *this) {
      VectorValues fakeDuals;
      fakeDuals.insert(factor->dualKey(), -Vector::Ones(factor->dim()));
      HessianFactor::shared_ptr actualLinearization(new HessianFactor(*factor->linearize(linearizationPoint)));
      HessianFactor::shared_ptr Hessian = boost::dynamic_pointer_cast<HessianFactor>(factor->multipliedHessian(linearizationPoint, fakeDuals));
      Hessian->linearTerm() = actualLinearization->linearTerm()/std::sqrt(actualLinearization->constantTerm());
      Hessian->constantTerm() = actualLinearization->constantTerm();
      linearizedGraph->push_back(Hessian);
    }
    return linearizedGraph;
  }
  
  double error(const Values & values) const {
    double total_error(0.0);

    for (const sharedFactor & factor : *this) {
      if (factor)
        total_error += std::abs(factor->unwhitenedError(values).sum());
    }

    return total_error;
  }
  
  GaussianFactorGraph::shared_ptr hessian(const Values & values) const {
    GaussianFactorGraph::shared_ptr hessians(new GaussianFactorGraph);
    for(NonlinearConstraint::shared_ptr factor: *this){
      VectorValues duals;
      duals.insert(factor->dualKey(), -Vector::Ones(factor->dim()));
      GaussianFactor::shared_ptr mH = factor->multipliedHessian(values, duals);
      hessians->push_back(mH);
    }
    return hessians;
  }
  
};
}
