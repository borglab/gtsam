/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSerializationHybrid.cpp
 * @brief   Unit tests for hybrid serialization
 * @author  Varun Agrawal
 * @date    January 2023
 */

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

using namespace serializationTestHelpers;

BOOST_CLASS_EXPORT_GUID(Factor, "gtsam_Factor");
BOOST_CLASS_EXPORT_GUID(HybridFactor, "gtsam_HybridFactor");
BOOST_CLASS_EXPORT_GUID(JacobianFactor, "gtsam_JacobianFactor");
BOOST_CLASS_EXPORT_GUID(GaussianConditional, "gtsam_GaussianConditional");
BOOST_CLASS_EXPORT_GUID(DiscreteConditional, "gtsam_DiscreteConditional");

BOOST_CLASS_EXPORT_GUID(DecisionTreeFactor, "gtsam_DecisionTreeFactor");
using ADT = AlgebraicDecisionTree<Key>;
BOOST_CLASS_EXPORT_GUID(ADT, "gtsam_AlgebraicDecisionTree");
BOOST_CLASS_EXPORT_GUID(ADT::Leaf, "gtsam_AlgebraicDecisionTree_Leaf");
BOOST_CLASS_EXPORT_GUID(ADT::Choice, "gtsam_AlgebraicDecisionTree_Choice")

BOOST_CLASS_EXPORT_GUID(GaussianMixtureFactor, "gtsam_GaussianMixtureFactor");
BOOST_CLASS_EXPORT_GUID(GaussianMixtureFactor::Factors,
                        "gtsam_GaussianMixtureFactor_Factors");
BOOST_CLASS_EXPORT_GUID(GaussianMixtureFactor::Factors::Leaf,
                        "gtsam_GaussianMixtureFactor_Factors_Leaf");
BOOST_CLASS_EXPORT_GUID(GaussianMixtureFactor::Factors::Choice,
                        "gtsam_GaussianMixtureFactor_Factors_Choice");

BOOST_CLASS_EXPORT_GUID(GaussianMixture, "gtsam_GaussianMixture");
BOOST_CLASS_EXPORT_GUID(GaussianMixture::Conditionals,
                        "gtsam_GaussianMixture_Conditionals");
BOOST_CLASS_EXPORT_GUID(GaussianMixture::Conditionals::Leaf,
                        "gtsam_GaussianMixture_Conditionals_Leaf");
BOOST_CLASS_EXPORT_GUID(GaussianMixture::Conditionals::Choice,
                        "gtsam_GaussianMixture_Conditionals_Choice");
// Needed since GaussianConditional::FromMeanAndStddev uses it
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

BOOST_CLASS_EXPORT_GUID(HybridBayesNet, "gtsam_HybridBayesNet");

/* ****************************************************************************/
// Test HybridGaussianFactor serialization.
TEST(HybridSerialization, HybridGaussianFactor) {
  const HybridGaussianFactor factor(JacobianFactor(X(0), I_3x3, Z_3x1));

  EXPECT(equalsObj<HybridGaussianFactor>(factor));
  EXPECT(equalsXML<HybridGaussianFactor>(factor));
  EXPECT(equalsBinary<HybridGaussianFactor>(factor));
}

/* ****************************************************************************/
// Test HybridDiscreteFactor serialization.
TEST(HybridSerialization, HybridDiscreteFactor) {
  DiscreteKeys discreteKeys{{M(0), 2}};
  const HybridDiscreteFactor factor(
      DecisionTreeFactor(discreteKeys, std::vector<double>{0.4, 0.6}));

  EXPECT(equalsObj<HybridDiscreteFactor>(factor));
  EXPECT(equalsXML<HybridDiscreteFactor>(factor));
  EXPECT(equalsBinary<HybridDiscreteFactor>(factor));
}

/* ****************************************************************************/
// Test GaussianMixtureFactor serialization.
TEST(HybridSerialization, GaussianMixtureFactor) {
  KeyVector continuousKeys{X(0)};
  DiscreteKeys discreteKeys{{M(0), 2}};

  auto A = Matrix::Zero(2, 1);
  auto b0 = Matrix::Zero(2, 1);
  auto b1 = Matrix::Ones(2, 1);
  auto f0 = boost::make_shared<JacobianFactor>(X(0), A, b0);
  auto f1 = boost::make_shared<JacobianFactor>(X(0), A, b1);
  std::vector<GaussianFactor::shared_ptr> factors{f0, f1};

  const GaussianMixtureFactor factor(continuousKeys, discreteKeys, factors);

  EXPECT(equalsObj<GaussianMixtureFactor>(factor));
  EXPECT(equalsXML<GaussianMixtureFactor>(factor));
  EXPECT(equalsBinary<GaussianMixtureFactor>(factor));
}

/* ****************************************************************************/
// Test HybridConditional serialization.
TEST(HybridSerialization, HybridConditional) {
  const DiscreteKey mode(M(0), 2);
  Matrix1 I = Matrix1::Identity();
  const auto conditional = boost::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 0.5));
  const HybridConditional hc(conditional);

  EXPECT(equalsObj<HybridConditional>(hc));
  EXPECT(equalsXML<HybridConditional>(hc));
  EXPECT(equalsBinary<HybridConditional>(hc));
}

/* ****************************************************************************/
// Test GaussianMixture serialization.
TEST(HybridSerialization, GaussianMixture) {
  const DiscreteKey mode(M(0), 2);
  Matrix1 I = Matrix1::Identity();
  const auto conditional0 = boost::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 0.5));
  const auto conditional1 = boost::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 3));
  const GaussianMixture gm({Z(0)}, {X(0)}, {mode},
                           {conditional0, conditional1});

  EXPECT(equalsObj<GaussianMixture>(gm));
  EXPECT(equalsXML<GaussianMixture>(gm));
  EXPECT(equalsBinary<GaussianMixture>(gm));
}

/* ****************************************************************************/
// Test HybridBayesNet serialization.
TEST(HybridSerialization, HybridBayesNet) {
  Switching s(2);
  Ordering ordering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet hbn = *(s.linearizedFactorGraph.eliminateSequential(ordering));

  EXPECT(equalsObj<HybridBayesNet>(hbn));
  EXPECT(equalsXML<HybridBayesNet>(hbn));
  EXPECT(equalsBinary<HybridBayesNet>(hbn));
}

/* ****************************************************************************/
// Test HybridBayesTree serialization.
TEST(HybridSerialization, HybridBayesTree) {
  Switching s(2);
  Ordering ordering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesTree hbt =
      *(s.linearizedFactorGraph.eliminateMultifrontal(ordering));

  EXPECT(equalsObj<HybridBayesTree>(hbt));
  EXPECT(equalsXML<HybridBayesTree>(hbt));
  EXPECT(equalsBinary<HybridBayesTree>(hbt));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
