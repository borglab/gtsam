/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testSerializtionDiscrete.cpp
 *
 *  @date January 2023
 *  @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

using Tree = gtsam::DecisionTree<string, int>;

BOOST_CLASS_EXPORT_GUID(Tree, "gtsam_DecisionTreeStringInt")
BOOST_CLASS_EXPORT_GUID(Tree::Leaf, "gtsam_DecisionTreeStringInt_Leaf")
BOOST_CLASS_EXPORT_GUID(Tree::Choice, "gtsam_DecisionTreeStringInt_Choice")

BOOST_CLASS_EXPORT_GUID(DecisionTreeFactor, "gtsam_DecisionTreeFactor");

using ADT = AlgebraicDecisionTree<Key>;
BOOST_CLASS_EXPORT_GUID(ADT, "gtsam_AlgebraicDecisionTree");
BOOST_CLASS_EXPORT_GUID(ADT::Leaf, "gtsam_AlgebraicDecisionTree_Leaf")
BOOST_CLASS_EXPORT_GUID(ADT::Choice, "gtsam_AlgebraicDecisionTree_Choice")

/* ****************************************************************************/
// Test DecisionTree serialization.
TEST(DiscreteSerialization, DecisionTree) {
  Tree tree({{"A", 2}}, std::vector<int>{1, 2});

  using namespace serializationTestHelpers;

  // Object roundtrip
  Tree outputObj = create<Tree>();
  roundtrip<Tree>(tree, outputObj);
  EXPECT(tree.equals(outputObj));

  // XML roundtrip
  Tree outputXml = create<Tree>();
  roundtripXML<Tree>(tree, outputXml);
  EXPECT(tree.equals(outputXml));

  // Binary roundtrip
  Tree outputBinary = create<Tree>();
  roundtripBinary<Tree>(tree, outputBinary);
  EXPECT(tree.equals(outputBinary));
}

/* ************************************************************************* */
// Check serialization for AlgebraicDecisionTree and the DecisionTreeFactor
TEST(DiscreteSerialization, DecisionTreeFactor) {
  using namespace serializationTestHelpers;

  DiscreteKey A(1, 2), B(2, 2), C(3, 2);

  DecisionTreeFactor::ADT tree(A & B & C, "1 5 3 7 2 6 4 8");
  EXPECT(equalsObj<DecisionTreeFactor::ADT>(tree));
  EXPECT(equalsXML<DecisionTreeFactor::ADT>(tree));
  EXPECT(equalsBinary<DecisionTreeFactor::ADT>(tree));

  DecisionTreeFactor f(A & B & C, "1 5 3 7 2 6 4 8");
  EXPECT(equalsObj<DecisionTreeFactor>(f));
  EXPECT(equalsXML<DecisionTreeFactor>(f));
  EXPECT(equalsBinary<DecisionTreeFactor>(f));
}

/* ************************************************************************* */
// Check serialization for DiscreteConditional & DiscreteDistribution
TEST(DiscreteSerialization, DiscreteConditional) {
  using namespace serializationTestHelpers;

  DiscreteKey A(Symbol('x', 1), 3);
  DiscreteConditional conditional(A % "1/2/2");

  EXPECT(equalsObj<DiscreteConditional>(conditional));
  EXPECT(equalsXML<DiscreteConditional>(conditional));
  EXPECT(equalsBinary<DiscreteConditional>(conditional));

  DiscreteDistribution P(A % "3/2/1");
  EXPECT(equalsObj<DiscreteDistribution>(P));
  EXPECT(equalsXML<DiscreteDistribution>(P));
  EXPECT(equalsBinary<DiscreteDistribution>(P));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
