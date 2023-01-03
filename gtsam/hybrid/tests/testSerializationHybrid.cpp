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
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/inference/Symbol.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;

using namespace serializationTestHelpers;

BOOST_CLASS_EXPORT_GUID(HybridFactor, "gtsam_HybridFactor");
BOOST_CLASS_EXPORT_GUID(JacobianFactor, "gtsam_JacobianFactor");

BOOST_CLASS_EXPORT_GUID(DecisionTreeFactor, "gtsam_DecisionTreeFactor");
using ADT = AlgebraicDecisionTree<Key>;
BOOST_CLASS_EXPORT_GUID(ADT, "gtsam_AlgebraicDecisionTree");
BOOST_CLASS_EXPORT_GUID(ADT::Leaf, "gtsam_DecisionTree_Leaf")
BOOST_CLASS_EXPORT_GUID(ADT::Choice, "gtsam_DecisionTree_Choice")

BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixtureFactor, "gtsam_GaussianMixtureFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixtureFactor::Factors, "gtsam_GaussianMixtureFactor_Factors")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixtureFactor::Factors::Leaf, "gtsam_GaussianMixtureFactor_Factors_Leaf")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixtureFactor::Factors::Choice, "gtsam_GaussianMixtureFactor_Factors_Choice")

// BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixture, "gtsam_GaussianMixture")
// BOOST_CLASS_EXPORT_GUID(gtsam::GaussianMixture::Conditionals,
//                         "gtsam_GaussianMixture_Conditionals")

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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
