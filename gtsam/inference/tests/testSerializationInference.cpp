/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationInference.cpp
 * @brief 
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <gtsam/inference/IndexConditionalOrdered.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/BayesTreeOrdered.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
TEST (Serialization, symbolic_graph) {
  // construct expected symbolic graph
  SymbolicFactorGraphOrdered sfg;
  sfg.push_factor(0);
  sfg.push_factor(0,1);
  sfg.push_factor(0,2);
  sfg.push_factor(2,1);

  EXPECT(equalsObj(sfg));
  EXPECT(equalsXML(sfg));
  EXPECT(equalsBinary(sfg));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bn) {
  IndexConditionalOrdered::shared_ptr x2(new IndexConditionalOrdered(1, 2, 0));
  IndexConditionalOrdered::shared_ptr l1(new IndexConditionalOrdered(2, 0));
  IndexConditionalOrdered::shared_ptr x1(new IndexConditionalOrdered(0));

  SymbolicBayesNetOrdered sbn;
  sbn.push_back(x2);
  sbn.push_back(l1);
  sbn.push_back(x1);

  EXPECT(equalsObj(sbn));
  EXPECT(equalsXML(sbn));
  EXPECT(equalsBinary(sbn));
}

/* ************************************************************************* */
TEST (Serialization, symbolic_bayes_tree ) {
  typedef BayesTreeOrdered<IndexConditionalOrdered> SymbolicBayesTree;
  static const Index _X_=0, _T_=1, _S_=2, _E_=3, _L_=4, _B_=5;
  IndexConditionalOrdered::shared_ptr
  B(new IndexConditionalOrdered(_B_)),
  L(new IndexConditionalOrdered(_L_, _B_)),
  E(new IndexConditionalOrdered(_E_, _L_, _B_)),
  S(new IndexConditionalOrdered(_S_, _L_, _B_)),
  T(new IndexConditionalOrdered(_T_, _E_, _L_)),
  X(new IndexConditionalOrdered(_X_, _E_));

  // Bayes Tree for Asia example
  SymbolicBayesTree bayesTree;
  SymbolicBayesTree::insert(bayesTree, B);
  SymbolicBayesTree::insert(bayesTree, L);
  SymbolicBayesTree::insert(bayesTree, E);
  SymbolicBayesTree::insert(bayesTree, S);
  SymbolicBayesTree::insert(bayesTree, T);
  SymbolicBayesTree::insert(bayesTree, X);

  EXPECT(equalsObj(bayesTree));
  EXPECT(equalsXML(bayesTree));
  EXPECT(equalsBinary(bayesTree));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
