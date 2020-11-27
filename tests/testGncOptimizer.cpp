/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGncOptimizer.cpp
 * @brief   Unit tests for GncOptimizer class
 * @author  Jingnan Shi
 * @author  Luca Carlone
 * @author  Frank Dellaert
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <tests/smallExample.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::L;
static double tol = 1e-7;

/* ************************************************************************* */
template <class BaseOptimizerParameters>
class GncParams {
public:

  /** See NonlinearOptimizerParams::verbosity */
  enum RobustLossType {
    GM /*Geman McClure*/, TLS /*Truncated least squares*/
  };

  // using BaseOptimizer = GaussNewtonOptimizer; // BaseOptimizerParameters::OptimizerType;

  GncParams(const BaseOptimizerParameters& baseOptimizerParams):
    baseOptimizerParams(baseOptimizerParams),
    lossType(GM),      /* default loss*/
    maxIterations(100), /* maximum number of iterations*/
    barcSq(1.0), /* a factor is considered an inlier if factor.error() < barcSq. Note that factor.error() whitens by the covariance*/
    muStep(1.4){}/* multiplicative factor to reduce/increase the mu in gnc */

  // default constructor
  GncParams(): baseOptimizerParams() {}

  BaseOptimizerParameters baseOptimizerParams;
  /// any other specific GNC parameters:
  RobustLossType lossType;
  size_t maxIterations;
  double barcSq;
  double muStep;

  void setLossType(RobustLossType type){ lossType = type; }
  void setMaxIterations(size_t maxIter){
    std::cout
    << "setMaxIterations: changing the max number of iterations might lead to less accurate solutions and is not recommended! "
    << std::endl;
    maxIterations = maxIter;
  }
  void setInlierThreshold(double inth){ barcSq = inth; }
  void setMuStep(double step){ muStep = step; }

  /// equals
  bool equals(const GncParams& other, double tol = 1e-9) const {
    return baseOptimizerParams.equals(other.baseOptimizerParams)
        && lossType == other.lossType
        && maxIterations == other.maxIterations
        && std::fabs(barcSq - other.barcSq) <= tol
        && std::fabs(muStep - other.muStep) <= tol;
  }

  /// print function
  void print(const std::string& str) const {
    std::cout << str << "\n";
    switch(lossType) {
    case GM: std::cout << "lossType: Geman McClure" << "\n"; break;
    default:
      throw std::runtime_error(
          "GncParams::print: unknown loss type.");
    }
    std::cout << "maxIterations: " << maxIterations << "\n";
    std::cout << "barcSq: " << barcSq << "\n";
    std::cout << "muStep: " << muStep << "\n";
    baseOptimizerParams.print(str);
  }
};

/* ************************************************************************* */
template<class GncParameters>
class GncOptimizer {
public:
  // types etc

private:
  NonlinearFactorGraph nfg_;
  Values state_;
  GncParameters params_;

public:
  GncOptimizer(const NonlinearFactorGraph& graph,
      const Values& initialValues, const GncParameters& params = GncParameters()) :
        nfg_(graph), state_(initialValues), params_(params) {
    // TODO: Check that all noise models are Gaussian
  }

  NonlinearFactorGraph getFactors() const { return NonlinearFactorGraph(nfg_); }
  Values getState() const { return Values(state_); }
  GncParameters getParams() const { return GncParameters(params_); }

  /// implement GNC main loop, including graduating nonconvexity with mu
  Values optimize() {
    // start by assuming all measurements are inliers
    Vector weights = Vector::Ones(nfg_.size());
    GaussNewtonOptimizer baseOptimizer(nfg_,state_);
    Values result = baseOptimizer.optimize();
    double mu = initializeMu();
    for(size_t iter=0; iter < params_.maxIterations; iter++){
      // weights update
      weights = calculateWeights(result, mu);

      // variable/values update
      NonlinearFactorGraph graph_iter = this->makeGraph(weights);
      GaussNewtonOptimizer baseOptimizer_iter(graph_iter, state_);
      Values result = baseOptimizer.optimize();

      // stopping condition
      if( checkMuConvergence(mu) ) { break; }

      // otherwise update mu
      mu = updateMu(mu);
    }
    return result;
  }

  /// initialize the gnc parameter mu such that loss is approximately convex
  double initializeMu() const {
    // compute largest error across all factors
    double rmax_sq = 0.0;
    for (size_t i = 0; i < nfg_.size(); i++) {
      if(nfg_[i]){
        rmax_sq = std::max(rmax_sq, nfg_[i]->error(state_));
      }
    }
    // set initial mu
    switch(params_.lossType) {
    case GncParameters::GM:
      return  2*rmax_sq / params_.barcSq; // initial mu
    default:
      throw std::runtime_error(
          "GncOptimizer::initializeMu: called with unknown loss type.");
    }
  }

  /// update the gnc parameter mu to gradually increase nonconvexity
  double updateMu(const double mu) const {
    switch(params_.lossType) {
    case GncParameters::GM:
      return  std::max(1.0 , mu / params_.muStep); // reduce mu, but saturate at 1
    default:
      throw std::runtime_error(
          "GncOptimizer::updateMu: called with unknown loss type.");
    }
  }

  /// check if we have reached the value of mu for which the surrogate loss matches the original loss
  bool checkMuConvergence(const double mu) const {
    switch(params_.lossType) {
    case GncParameters::GM:
      return std::fabs(mu - 1.0) < 1e-9; // mu=1 recovers the original GM function
    default:
      throw std::runtime_error(
          "GncOptimizer::checkMuConvergence: called with unknown loss type.");
    }
  }

  /// create a graph where each factor is weighted by the gnc weights
  NonlinearFactorGraph makeGraph(const Vector& weights) const {
   return NonlinearFactorGraph(nfg_);
  }

  /// calculate gnc weights
  Vector calculateWeights(const Values currentEstimate, const double mu){
    Vector weights = Vector::Ones(nfg_.size());
    return weights;
  }
};


/* ************************************************************************* */
TEST(GncOptimizer, gncParamsConstructor) {

  //check params are correctly parsed
  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams1(lmParams);
  CHECK(lmParams.equals(gncParams1.baseOptimizerParams));

  // check also default constructor
  GncParams<LevenbergMarquardtParams> gncParams1b;
  CHECK(lmParams.equals(gncParams1b.baseOptimizerParams));

  // and check params become different if we change lmParams
  lmParams.setVerbosity("DELTA");
  CHECK(! lmParams.equals(gncParams1.baseOptimizerParams));

  // and same for GN
  GaussNewtonParams gnParams;
  GncParams<GaussNewtonParams> gncParams2(gnParams);
  CHECK(gnParams.equals(gncParams2.baseOptimizerParams));

  // check default constructor
  GncParams<GaussNewtonParams> gncParams2b;
  CHECK(gnParams.equals(gncParams2b.baseOptimizerParams));

  // change something at the gncParams level
  GncParams<GaussNewtonParams> gncParams2c(gncParams2b);
  gncParams2c.setLossType(GncParams<GaussNewtonParams>::RobustLossType::TLS);
  CHECK(! gncParams2c.equals(gncParams2b.baseOptimizerParams));
}

/* ************************************************************************* */
TEST(GncOptimizer, gncConstructor) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph(); // just a unary factor on a 2D point

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

  CHECK(gnc.getFactors().equals(fg));
  CHECK(gnc.getState().equals(initial));
  CHECK(gnc.getParams().equals(gncParams));
}

///* ************************************************************************* */
//TEST(GncOptimizer, calculateWeights) {
//}
//
///* ************************************************************************* */
//TEST(GncOptimizer, calculateWeights) {
//}
//
///* ************************************************************************* */
//TEST(GncOptimizer, copyGraph) {
//}

/* ************************************************************************* *
TEST(GncOptimizer, makeGraph) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph(); // just a unary factor on a 2D point

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

//  NonlinearFactorGraph actual = gnc.makeGraph(initial);
}

/* ************************************************************************* */
TEST(GncOptimizer, optimize) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

  gncParams.print("");

  Values actual = gnc.optimize();
  DOUBLES_EQUAL(0, fg.error(actual), tol);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
