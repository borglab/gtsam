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
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
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

BOOST_CLASS_EXPORT_GUID(HybridGaussianFactor, "gtsam_HybridGaussianFactor");
BOOST_CLASS_EXPORT_GUID(HybridGaussianFactor::Factors,
                        "gtsam_HybridGaussianFactor_Factors");
BOOST_CLASS_EXPORT_GUID(HybridGaussianFactor::Factors::Leaf,
                        "gtsam_HybridGaussianFactor_Factors_Leaf");
BOOST_CLASS_EXPORT_GUID(HybridGaussianFactor::Factors::Choice,
                        "gtsam_HybridGaussianFactor_Factors_Choice");

BOOST_CLASS_EXPORT_GUID(HybridGaussianConditional,
                        "gtsam_HybridGaussianConditional");
BOOST_CLASS_EXPORT_GUID(HybridGaussianConditional::Conditionals,
                        "gtsam_HybridGaussianConditional_Conditionals");
BOOST_CLASS_EXPORT_GUID(HybridGaussianConditional::Conditionals::Leaf,
                        "gtsam_HybridGaussianConditional_Conditionals_Leaf");
BOOST_CLASS_EXPORT_GUID(HybridGaussianConditional::Conditionals::Choice,
                        "gtsam_HybridGaussianConditional_Conditionals_Choice");
// Needed since GaussianConditional::FromMeanAndStddev uses it
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

BOOST_CLASS_EXPORT_GUID(HybridBayesNet, "gtsam_HybridBayesNet");

/* ****************************************************************************/
// Test HybridGaussianFactor serialization.
TEST(HybridSerialization, HybridGaussianFactor) {
  KeyVector continuousKeys{X(0)};
  DiscreteKey discreteKey{M(0), 2};

  auto A = Matrix::Zero(2, 1);
  auto b0 = Matrix::Zero(2, 1);
  auto b1 = Matrix::Ones(2, 1);
  auto f0 = std::make_shared<JacobianFactor>(X(0), A, b0);
  auto f1 = std::make_shared<JacobianFactor>(X(0), A, b1);
  std::vector<GaussianFactor::shared_ptr> factors{f0, f1};

  const HybridGaussianFactor factor(continuousKeys, discreteKey, factors);

  EXPECT(equalsObj<HybridGaussianFactor>(factor));
  EXPECT(equalsXML<HybridGaussianFactor>(factor));
  EXPECT(equalsBinary<HybridGaussianFactor>(factor));
}

/* ****************************************************************************/
// Test HybridConditional serialization.
TEST(HybridSerialization, HybridConditional) {
  const DiscreteKey mode(M(0), 2);
  Matrix1 I = Matrix1::Identity();
  const auto conditional = std::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 0.5));
  const HybridConditional hc(conditional);

  EXPECT(equalsObj<HybridConditional>(hc));
  EXPECT(equalsXML<HybridConditional>(hc));
  EXPECT(equalsBinary<HybridConditional>(hc));
}

/* ****************************************************************************/
// Test HybridGaussianConditional serialization.
TEST(HybridSerialization, HybridGaussianConditional) {
  const DiscreteKey mode(M(0), 2);
  Matrix1 I = Matrix1::Identity();
  const auto conditional0 = std::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 0.5));
  const auto conditional1 = std::make_shared<GaussianConditional>(
      GaussianConditional::FromMeanAndStddev(Z(0), I, X(0), Vector1(0), 3));
  const HybridGaussianConditional gm(mode, {conditional0, conditional1});

  EXPECT(equalsObj<HybridGaussianConditional>(gm));
  EXPECT(equalsXML<HybridGaussianConditional>(gm));
  EXPECT(equalsBinary<HybridGaussianConditional>(gm));
}

/* ****************************************************************************/
// Test HybridBayesNet serialization.
TEST(HybridSerialization, HybridBayesNet) {
  Switching s(2);
  HybridBayesNet hbn = *(s.linearizedFactorGraph.eliminateSequential());

  EXPECT(equalsObj<HybridBayesNet>(hbn));
  EXPECT(equalsXML<HybridBayesNet>(hbn));
  EXPECT(equalsBinary<HybridBayesNet>(hbn));
}

/* ****************************************************************************/
// Test HybridBayesTree serialization.
TEST(HybridSerialization, HybridBayesTree) {
  Switching s(2);
  HybridBayesTree hbt = *(s.linearizedFactorGraph.eliminateMultifrontal());

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
