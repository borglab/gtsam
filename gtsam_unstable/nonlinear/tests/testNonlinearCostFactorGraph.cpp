/**
 * @file 	 testNonlinearCostFactorGraph.cpp
 * @brief
 * @author Ivan Dario Jimenez
 * @date 	 Oct 26, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/nonlinear/SQPLineSearch2.h>
#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearInequalityConstraint.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;

namespace hessianOfCost {
  struct costError{
    Vector operator()(const double & x1, const double & x2,
                      boost::optional<Matrix&> H1 = boost::none,
                      boost::optional<Matrix&> H2 = boost::none){
      if(H1){
        *H1 = 4*std::pow((x1 -1),3)*I_1x1;
      }
      if(H2){
        *H2 = 4*std::pow((x2 -1),3)*I_1x1;
      }
      return (std::pow(x1 -1, 4) + std::pow(x2-1, 4))*I_1x1;
    }
  };
  typedef NonlinearEqualityConstraint2<double,double, costError> cost;
  
  class costWithHessian : public cost {
  public:
    costWithHessian(Key j1, Key j2, Key dualKey, size_t constraintDim = 1) :
      NonlinearEqualityConstraint2(j1, j2, dualKey, constraintDim) {}
  
    costWithHessian(const SharedNoiseModel &noiseModel, Key j1, Key j2, Key dualKey) :
      NonlinearEqualityConstraint2(noiseModel, j1, j2, dualKey) {}

  private:
    void evaluateHessians(const X1 &x1, const X2 &x2,
                          std::vector<Matrix> &G11,
                          std::vector<Matrix> &G12,
                          std::vector<Matrix> &G22) const override {
      G11.push_back(I_1x1*12*std::pow(x1-1, 2));
      G22.push_back(I_1x1*12*std::pow(x2-1, 2));
      G12.push_back(Z_1x1);
    }
  };
}


TEST(CostHessianCalculation, HessianOfCostNoFunction){
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  Matrix33 answer;
  answer << 48, 0, 0, 0, 48, 0, 0, 0, 0;
  NonlinearCostFactorGraph cost;
  cost.push_back(hessianOfCost::cost(X, Y, D));
  Values linpoint;
  linpoint.insert(X, 3.0);
  linpoint.insert(Y, 3.0);
  GaussianFactorGraph::shared_ptr cost_hessian_factor = cost.hessian(linpoint);
  Matrix AugmentedHessian = cost_hessian_factor->augmentedHessian();
  CHECK(assert_equal(answer, cost_hessian_factor->augmentedHessian()));
}

TEST(CostHessianCalculation, HessianOfCostWithFunction){
  Key X(Symbol('X',1)) , Y(Symbol('Y',1)), D(Symbol('D',1));
  Matrix33 answer;
  answer << 48, 0, 0, 0, 48, 0, 0, 0, 0;
  NonlinearCostFactorGraph cost;
  cost.push_back(hessianOfCost::costWithHessian(X, Y, D));
  Values linpoint;
  linpoint.insert(X, 3.0);
  linpoint.insert(Y, 3.0);
  GaussianFactorGraph::shared_ptr cost_hessian_factor = cost.hessian(linpoint);
  CHECK(assert_equal(answer, cost_hessian_factor->augmentedHessian()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}