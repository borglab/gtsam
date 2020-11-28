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
 *
 * Implementation of the paper: Yang, Antonante, Tzoumas, Carlone, "Graduated Non-Convexity for Robust Spatial Perception:
 * From Non-Minimal Solvers to Global Outlier Rejection", RAL, 2020. (arxiv version: https://arxiv.org/pdf/1909.08605.pdf)
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
        state_(initialValues), params_(params) {

    // make sure all noiseModels are Gaussian or convert to Gaussian
    nfg_.resize(graph.size());
    for (size_t i = 0; i < graph.size(); i++) {
      if(graph[i]){
        NoiseModelFactor::shared_ptr factor = boost::dynamic_pointer_cast<NoiseModelFactor>(graph[i]);
        noiseModel::Robust::shared_ptr robust = boost::dynamic_pointer_cast<noiseModel::Robust>(factor->noiseModel());
        if(robust){ // if the factor has a robust loss, we have to change it:
          SharedNoiseModel gaussianNoise = robust->noise();
          NoiseModelFactor::shared_ptr gaussianFactor = factor->cloneWithNewNoiseModel(gaussianNoise);
          nfg_[i] = gaussianFactor;
        } else{ // else we directly push it back
          nfg_[i] = factor;
        }
      }
    }
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
      NonlinearFactorGraph graph_iter = this->makeWeightedGraph(weights);
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
  NonlinearFactorGraph makeWeightedGraph(const Vector& weights) const {
    // make sure all noiseModels are Gaussian or convert to Gaussian
    NonlinearFactorGraph newGraph;
    newGraph.resize(nfg_.size());
    for (size_t i = 0; i < nfg_.size(); i++) {
      if(nfg_[i]){
        NoiseModelFactor::shared_ptr factor = boost::dynamic_pointer_cast<NoiseModelFactor>(nfg_[i]);
        noiseModel::Gaussian::shared_ptr noiseModel = boost::dynamic_pointer_cast<noiseModel::Gaussian>(factor->noiseModel());
        if(noiseModel){
          Matrix newInfo = weights[i] * noiseModel->information();
          SharedNoiseModel newNoiseModel = noiseModel::Gaussian::Information(newInfo);
          newGraph[i] = factor->cloneWithNewNoiseModel(newNoiseModel);
        }else{
          throw std::runtime_error(
                    "GncOptimizer::makeWeightedGraph: unexpected non-Gaussian noise model.");
        }
      }
    }
    return newGraph;
  }

  /// calculate gnc weights
  Vector calculateWeights(const Values currentEstimate, const double mu){
    Vector weights = Vector::Zero(nfg_.size());
    switch(params_.lossType) {
    case GncParameters::GM: // use eq (12) in GNC paper
      for (size_t k = 0; k < nfg_.size(); k++) {
        if(nfg_[k]){
          double u2_k = nfg_[k]->error(currentEstimate); // squared (and whitened) residual
          weights[k] = std::pow( ( mu*params_.barcSq )/( u2_k + mu*params_.barcSq ) , 2);
        }
      }
      return weights;
    default:
      throw std::runtime_error(
          "GncOptimizer::calculateWeights: called with unknown loss type.");
    }
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

/* ************************************************************************* */
TEST(GncOptimizer, gncConstructorWithRobustGraphAsInput) {
  // simple graph with Gaussian noise model
  auto fg = example::sharedNonRobustFactorGraphWithOutliers();
  // same graph with robust noise model
  auto fg_robust = example::sharedRobustFactorGraphWithOutliers();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg_robust, initial, gncParams);
//  fg.print("fg\n");
//  fg_robust.print("fg_robust\n");
//  gnc.getFactors().print("gnc\n");
  // make sure that when parsing the graph is transformed into one without robust loss
  CHECK( fg.equals(gnc.getFactors()) );
}

/* ************************************************************************* */
TEST(GncOptimizer, initializeMu) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  gncParams.setLossType(GncParams<LevenbergMarquardtParams>::RobustLossType::GM);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);
  EXPECT_DOUBLES_EQUAL(gnc.initializeMu(), 2 * 198.999, 1e-3); // according to rmk 5 in the gnc paper: m0 = 2 rmax^2 / barcSq (barcSq=1 in this example)
}

/* ************************************************************************* */
TEST(GncOptimizer, updateMu) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  gncParams.setLossType(GncParams<LevenbergMarquardtParams>::RobustLossType::GM);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

  double mu = 5.0;
  EXPECT_DOUBLES_EQUAL(gnc.updateMu(mu), mu / 1.4, tol);

  // check it correctly saturates to 1 for GM
  mu = 1.2;
  EXPECT_DOUBLES_EQUAL(gnc.updateMu(mu), 1.0, tol);
}

/* ************************************************************************* */
TEST(GncOptimizer, checkMuConvergence) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  gncParams.setLossType(GncParams<LevenbergMarquardtParams>::RobustLossType::GM);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

  double mu = 1.0;
  CHECK(gnc.checkMuConvergence(mu));
}

/* ************************************************************************* */
TEST(GncOptimizer, calculateWeights) {
  // has to have Gaussian noise models !
    auto fg = example::sharedNonRobustFactorGraphWithOutliers();

    Point2 p0(0, 0);
    Values initial;
    initial.insert(X(1), p0);

    // we have 4 factors, 3 with zero errors (inliers), 1 with error 50 = 0.5 * 1/sigma^2 || [1;0] - [0;0] ||^2 (outlier)
    Vector weights_expected = Vector::Zero(4);
    weights_expected[0] = 1.0; // zero error
    weights_expected[1] = 1.0; // zero error
    weights_expected[2] = 1.0; // zero error
    weights_expected[3] = std::pow(1.0 / (50.0 + 1.0),2); // outlier, error = 50

    GaussNewtonParams gnParams;
    GncParams<GaussNewtonParams> gncParams(gnParams);
    auto gnc = GncOptimizer<GncParams<GaussNewtonParams>>(fg, initial, gncParams);
    double mu = 1.0;
    Vector weights_actual = gnc.calculateWeights(initial,mu);
    CHECK(assert_equal(weights_expected, weights_actual, tol));

    mu = 2.0;
    double barcSq = 5.0;
    weights_expected[3] = std::pow(mu*barcSq / (50.0 + mu*barcSq),2); // outlier, error = 50
    gncParams.setInlierThreshold(barcSq);
    auto gnc2 = GncOptimizer<GncParams<GaussNewtonParams>>(fg, initial, gncParams);
    weights_actual = gnc2.calculateWeights(initial,mu);
    CHECK(assert_equal(weights_expected, weights_actual, tol));
}

/* ************************************************************************* */
TEST(GncOptimizer, makeWeightedGraph) {
  // create original factor
    double sigma1 = 0.1;
    NonlinearFactorGraph nfg = example::nonlinearFactorGraphWithGivenSigma(sigma1);

    // create expected
    double sigma2 = 10;
    NonlinearFactorGraph expected = example::nonlinearFactorGraphWithGivenSigma(sigma2);

    // create weights
    Vector weights = Vector::Ones(1); // original info:1/0.1^2 = 100. New info: 1/10^2 = 0.01. Ratio is 10-4
    weights[0] = 1e-4;

    // create actual
    Point2 p0(3, 3);
    Values initial;
    initial.insert(X(1), p0);

    LevenbergMarquardtParams lmParams;
    GncParams<LevenbergMarquardtParams> gncParams(lmParams);
    auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(nfg, initial, gncParams);
    NonlinearFactorGraph actual = gnc.makeWeightedGraph(weights);

    // check it's all good
    CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GncOptimizer, optimizeSimple) {
  // has to have Gaussian noise models !
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 p0(3, 3);
  Values initial;
  initial.insert(X(1), p0);

  LevenbergMarquardtParams lmParams;
  GncParams<LevenbergMarquardtParams> gncParams(lmParams);
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(fg, initial, gncParams);

  Values actual = gnc.optimize();
  DOUBLES_EQUAL(0, fg.error(actual), tol);
}

/* ************************************************************************* */
TEST(GncOptimizer, optimize) {
  // has to have Gaussian noise models !
  auto fg = example::sharedNonRobustFactorGraphWithOutliers();

  Point2 p0(1, 0);
  Values initial;
  initial.insert(X(1), p0);

  // try with nonrobust cost function and standard GN
  GaussNewtonParams gnParams;
  GaussNewtonOptimizer gn(fg, initial, gnParams);
  Values gn_results = gn.optimize();
  // converges to incorrect point due to lack of robustness to an outlier, ideal solution is Point2(0,0)
  CHECK(assert_equal(Point2(0.25,0.0), gn_results.at<Point2>(X(1)), 1e-3));

  // try with robust loss function and standard GN
  auto fg_robust = example::sharedRobustFactorGraphWithOutliers(); // same as fg, but with factors wrapped in Geman McClure losses
  GaussNewtonOptimizer gn2(fg_robust, initial, gnParams);
  Values gn2_results = gn2.optimize();
  // converges to incorrect point, this time due to the nonconvexity of the loss
  CHECK(assert_equal(Point2(0.999706,0.0), gn2_results.at<Point2>(X(1)), 1e-3));

  // .. but graduated nonconvexity ensures both robustness and convergence in the face of nonconvexity
  GncParams<GaussNewtonParams> gncParams(gnParams);
  auto gnc = GncOptimizer<GncParams<GaussNewtonParams>>(fg, initial, gncParams);
  Values gnc_result = gnc.optimize();
  CHECK(assert_equal(Point2(0.0,0.0), gnc_result.at<Point2>(X(1)), 1e-3));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
