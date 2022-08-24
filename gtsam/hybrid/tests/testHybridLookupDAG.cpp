/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file testHybridLookupDAG.cpp
 *  @date Aug, 2022
 *  @author Shangjie Xue
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridLookupDAG.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

TEST(HybridLookupTable, basics) {
  // create a conditional gaussian node
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 2;
  S1(0, 1) = 3;
  S1(1, 1) = 4;

  Matrix S2(2, 2);
  S2(0, 0) = 6;
  S2(1, 0) = 0.2;
  S2(0, 1) = 8;
  S2(1, 1) = 0.4;

  Matrix R1(2, 2);
  R1(0, 0) = 0.1;
  R1(1, 0) = 0.3;
  R1(0, 1) = 0.0;
  R1(1, 1) = 0.34;

  Matrix R2(2, 2);
  R2(0, 0) = 0.1;
  R2(1, 0) = 0.3;
  R2(0, 1) = 0.0;
  R2(1, 1) = 0.34;

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  Vector2 d1(0.2, 0.5), d2(0.5, 0.2);

  auto conditional0 = boost::make_shared<GaussianConditional>(X(1), d1, R1,
                                                              X(2), S1, model),
       conditional1 = boost::make_shared<GaussianConditional>(X(1), d2, R2,
                                                              X(2), S2, model);

  // Create decision tree
  DiscreteKey m1(1, 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  //   GaussianMixture mixtureFactor2({X(1)}, {X(2)}, {m1}, conditionals);

  boost::shared_ptr<GaussianMixture> mixtureFactor(
      new GaussianMixture({X(1)}, {X(2)}, {m1}, conditionals));

  HybridConditional hc(mixtureFactor);

  GaussianMixture::Conditionals conditional2 =
      boost::static_pointer_cast<GaussianMixture>(hc.inner())->conditionals();

  DiscreteValues dv;
  dv[1] = 1;

  VectorValues cv;
  cv.insert(X(2), Vector2(0.0, 0.0));

  HybridValues hv(dv, cv);

  //   std::cout << conditional2(values).markdown();
  EXPECT(assert_equal(*conditional2(dv), *conditionals(dv), 1e-6));
  EXPECT(conditional2(dv) == conditionals(dv));
  HybridLookupTable hlt(hc);

  //   hlt.argmaxInPlace(&hv);

  HybridLookupDAG dag;
  dag.push_back(hlt);
  dag.argmax(hv);

  //   HybridBayesNet hbn;
  //   hbn.push_back(hc);
  //   hbn.optimize();
}

TEST(HybridLookupTable, hybrid_argmax) {
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 0;
  S1(0, 1) = 0;
  S1(1, 1) = 1;

  Vector2 d1(0.2, 0.5), d2(-0.5, 0.6);

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  auto conditional0 =
           boost::make_shared<GaussianConditional>(X(1), d1, S1, model),
       conditional1 =
           boost::make_shared<GaussianConditional>(X(1), d2, S1, model);

  DiscreteKey m1(1, 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  boost::shared_ptr<GaussianMixture> mixtureFactor(
      new GaussianMixture({X(1)}, {}, {m1}, conditionals));

  HybridConditional hc(mixtureFactor);

  DiscreteValues dv;
  dv[1] = 1;
  VectorValues cv;
  // cv.insert(X(2),Vector2(0.0, 0.0));
  HybridValues hv(dv, cv);

  HybridLookupTable hlt(hc);

  hlt.argmaxInPlace(&hv);

  EXPECT(assert_equal(hv.at(X(1)), d2));
}

TEST(HybridLookupTable, discrete_argmax) {
  DiscreteKey X(0, 2), Y(1, 2);

  auto conditional = boost::make_shared<DiscreteConditional>(X | Y = "0/1 3/2");

  HybridConditional hc(conditional);

  HybridLookupTable hlt(hc);

  DiscreteValues dv;
  dv[1] = 0;
  VectorValues cv;
  // cv.insert(X(2),Vector2(0.0, 0.0));
  HybridValues hv(dv, cv);

  hlt.argmaxInPlace(&hv);

  EXPECT(assert_equal(hv.atDiscrete(0), 1));

  DecisionTreeFactor f1(X, "2 3");
  auto conditional2 = boost::make_shared<DiscreteConditional>(1, f1);

  HybridConditional hc2(conditional2);

  HybridLookupTable hlt2(hc2);

  HybridValues hv2;

  hlt2.argmaxInPlace(&hv2);

  EXPECT(assert_equal(hv2.atDiscrete(0), 1));
}

TEST(HybridLookupTable, gaussian_argmax) {
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 0;
  S1(0, 1) = 0;
  S1(1, 1) = 1;

  Vector2 d1(0.2, 0.5), d2(-0.5, 0.6);

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  auto conditional =
      boost::make_shared<GaussianConditional>(X(1), d1, S1, X(2), -S1, model);

  HybridConditional hc(conditional);

  HybridLookupTable hlt(hc);

  DiscreteValues dv;
  // dv[1]=0;
  VectorValues cv;
  cv.insert(X(2), d2);
  HybridValues hv(dv, cv);

  hlt.argmaxInPlace(&hv);

  EXPECT(assert_equal(hv.at(X(1)), d1 + d2));
}

TEST(HybridLookupDAG, argmax) {
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 0;
  S1(0, 1) = 0;
  S1(1, 1) = 1;

  Vector2 d1(0.2, 0.5), d2(-0.5, 0.6);

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  auto conditional0 =
           boost::make_shared<GaussianConditional>(X(2), d1, S1, model),
       conditional1 =
           boost::make_shared<GaussianConditional>(X(2), d2, S1, model);

  DiscreteKey m1(1, 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  boost::shared_ptr<GaussianMixture> mixtureFactor(
      new GaussianMixture({X(2)}, {}, {m1}, conditionals));
  HybridConditional hc2(mixtureFactor);
  HybridLookupTable hlt2(hc2);

  auto conditional2 =
      boost::make_shared<GaussianConditional>(X(1), d1, S1, X(2), -S1, model);

  HybridConditional hc1(conditional2);
  HybridLookupTable hlt1(hc1);

  DecisionTreeFactor f1(m1, "2 3");
  auto discrete_conditional = boost::make_shared<DiscreteConditional>(1, f1);

  HybridConditional hc3(discrete_conditional);
  HybridLookupTable hlt3(hc3);

  HybridLookupDAG dag;
  dag.push_back(hlt1);
  dag.push_back(hlt2);
  dag.push_back(hlt3);
  auto hv = dag.argmax();

  EXPECT(assert_equal(hv.atDiscrete(1), 1));
  EXPECT(assert_equal(hv.at(X(2)), d2));
  EXPECT(assert_equal(hv.at(X(1)), d2 + d1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
