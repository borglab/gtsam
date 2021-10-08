"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Factor Graphs.
Author: Frank Dellaert
"""

import unittest
import numpy as np

import gtsam
from gtsam import DiscreteFactorGraph
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

# #include <gtsam/discrete/DiscreteFactor.h>
# #include <gtsam/discrete/DiscreteFactorGraph.h>
# #include <gtsam/discrete/DiscreteEliminationTree.h>
# #include <gtsam/discrete/DiscreteBayesTree.h>
# #include <gtsam/inference/BayesNet.h>

# #include <CppUnitLite/TestHarness.h>

# #include <boost/assign/std/map.hpp>
# using namespace boost::assign

# using namespace std
# using namespace gtsam

from gtsam import DiscreteKeys, DiscreteFactorGraph

class TestDiscreteFactorGraph(GtsamTestCase):
    """Tests for Discrete Factor Graphs."""

    def test_evaluation(self):
        # Three keys P1 and P2
        P1 = (0, 2)
        P2 = (1, 2)
        P3 = (2, 3)

        # Create the DiscreteFactorGraph
        graph = DiscreteFactorGraph()
        graph.add(P1, "0.9 0.3")
        graph.add(P2, "0.9 0.6")

        # NOTE(fan): originally is an operator overload in C++ &
        def discrete_and(a, b):
            dks = DiscreteKeys()
            dks.push_back(a)
            dks.push_back(b)
            return dks

        graph.add(discrete_and(P1, P2), "4 1 10 4")

        print(graph)

#   # Instantiate Values
#   DiscreteFactor::Values values
#   values[0] = 1
#   values[1] = 1

#   # Check if graph evaluation works ( 0.3*0.6*4 )
#   EXPECT_DOUBLES_EQUAL( .72, graph(values), 1e-9)

#   # Creating a new test with third node and adding unary and ternary factors on it
#   graph.add(P3, "0.9 0.2 0.5")
#   graph.add(P1 & P2 & P3, "1 2 3 4 5 6 7 8 9 10 11 12")

#   # Below values lead to selecting the 8th index in the ternary factor table
#   values[0] = 1
#   values[1] = 0
#   values[2] = 1

#   # Check if graph evaluation works (0.3*0.9*1*0.2*8)
#   EXPECT_DOUBLES_EQUAL( 4.32, graph(values), 1e-9)

#   # Below values lead to selecting the 3rd index in the ternary factor table
#   values[0] = 0
#   values[1] = 1
#   values[2] = 0

#   # Check if graph evaluation works (0.9*0.6*1*0.9*4)
#   EXPECT_DOUBLES_EQUAL( 1.944, graph(values), 1e-9)

#   # Check if graph product works
#   DecisionTreeFactor product = graph.product()
#   EXPECT_DOUBLES_EQUAL( 1.944, product(values), 1e-9)
# }

# /* ************************************************************************* */
# TEST( DiscreteFactorGraph, test)
# {
#   # Declare keys and ordering
#   DiscreteKey C(0,2), B(1,2), A(2,2)

#   # A simple factor graph (A)-fAC-(C)-fBC-(B)
#   # with smoothness priors
#   DiscreteFactorGraph graph
#   graph.add(A & C, "3 1 1 3")
#   graph.add(C & B, "3 1 1 3")

#   # Test EliminateDiscrete
#   # FIXME: apparently Eliminate returns a conditional rather than a net
#   Ordering frontalKeys
#   frontalKeys += Key(0)
#   DiscreteConditional::shared_ptr conditional
#   DecisionTreeFactor::shared_ptr newFactor
#   boost::tie(conditional, newFactor) = EliminateDiscrete(graph, frontalKeys)

#   # Check Bayes net
#   CHECK(conditional)
#   DiscreteBayesNet expected
#   Signature signature((C | B, A) = "9/1 1/1 1/1 1/9")
#   # cout << signature << endl
#   DiscreteConditional expectedConditional(signature)
#   EXPECT(assert_equal(expectedConditional, *conditional))
#   expected.add(signature)

#   # Check Factor
#   CHECK(newFactor)
#   DecisionTreeFactor expectedFactor(B & A, "10 6 6 10")
#   EXPECT(assert_equal(expectedFactor, *newFactor))

#   # add conditionals to complete expected Bayes net
#   expected.add(B | A = "5/3 3/5")
#   expected.add(A % "1/1")
#   #  GTSAM_PRINT(expected)

#   # Test elimination tree
#   Ordering ordering
#   ordering += Key(0), Key(1), Key(2)
#   DiscreteEliminationTree etree(graph, ordering)
#   DiscreteBayesNet::shared_ptr actual
#   DiscreteFactorGraph::shared_ptr remainingGraph
#   boost::tie(actual, remainingGraph) = etree.eliminate(&EliminateDiscrete)
#   EXPECT(assert_equal(expected, *actual))

# #  # Test solver
# #  DiscreteBayesNet::shared_ptr actual2 = solver.eliminate()
# #  EXPECT(assert_equal(expected, *actual2))

#   # Test optimization
#   DiscreteFactor::Values expectedValues
#   insert(expectedValues)(0, 0)(1, 0)(2, 0)
#   DiscreteFactor::sharedValues actualValues = graph.optimize()
#   EXPECT(assert_equal(expectedValues, *actualValues))
# }

# /* ************************************************************************* */
# TEST( DiscreteFactorGraph, testMPE)
# {
#   # Declare a bunch of keys
#   DiscreteKey C(0,2), A(1,2), B(2,2)

#   # Create Factor graph
#   DiscreteFactorGraph graph
#   graph.add(C & A, "0.2 0.8 0.3 0.7")
#   graph.add(C & B, "0.1 0.9 0.4 0.6")
#   #  graph.product().print()
#   #  DiscreteSequentialSolver(graph).eliminate()->print()

#   DiscreteFactor::sharedValues actualMPE = graph.optimize()

#   DiscreteFactor::Values expectedMPE
#   insert(expectedMPE)(0, 0)(1, 1)(2, 1)
#   EXPECT(assert_equal(expectedMPE, *actualMPE))
# }

# /* ************************************************************************* */
# TEST( DiscreteFactorGraph, testMPE_Darwiche09book_p244)
# {
#   # The factor graph in Darwiche09book, page 244
#   DiscreteKey A(4,2), C(3,2), S(2,2), T1(0,2), T2(1,2)

#   # Create Factor graph
#   DiscreteFactorGraph graph
#   graph.add(S, "0.55 0.45")
#   graph.add(S & C, "0.05 0.95 0.01 0.99")
#   graph.add(C & T1, "0.80 0.20 0.20 0.80")
#   graph.add(S & C & T2, "0.80 0.20 0.20 0.80 0.95 0.05 0.05 0.95")
#   graph.add(T1 & T2 & A, "1 0 0 1 0 1 1 0")
#   graph.add(A, "1 0")# evidence, A = yes (first choice in Darwiche)
#   #graph.product().print("Darwiche-product")
#   #  graph.product().potentials().dot("Darwiche-product")
#   #  DiscreteSequentialSolver(graph).eliminate()->print()

#   DiscreteFactor::Values expectedMPE
#   insert(expectedMPE)(4, 0)(2, 0)(3, 1)(0, 1)(1, 1)

#   # Use the solver machinery.
#   DiscreteBayesNet::shared_ptr chordal = graph.eliminateSequential()
#   DiscreteFactor::sharedValues actualMPE = chordal->optimize()
#   EXPECT(assert_equal(expectedMPE, *actualMPE))
# #  DiscreteConditional::shared_ptr root = chordal->back()
# #  EXPECT_DOUBLES_EQUAL(0.4, (*root)(*actualMPE), 1e-9)

#   # Let us create the Bayes tree here, just for fun, because we don't use it now
# #  typedef JunctionTreeOrdered<DiscreteFactorGraph> JT
# #  GenericMultifrontalSolver<DiscreteFactor, JT> solver(graph)
# #  BayesTreeOrdered<DiscreteConditional>::shared_ptr bayesTree = solver.eliminate(&EliminateDiscrete)
# ##  bayesTree->print("Bayes Tree")
# #  EXPECT_LONGS_EQUAL(2,bayesTree->size())

#   Ordering ordering
#   ordering += Key(0),Key(1),Key(2),Key(3),Key(4)
#   DiscreteBayesTree::shared_ptr bayesTree = graph.eliminateMultifrontal(ordering)
#   #  bayesTree->print("Bayes Tree")
#   EXPECT_LONGS_EQUAL(2,bayesTree->size())

# #ifdef OLD
# # Create the elimination tree manually
# VariableIndexOrdered structure(graph)
# typedef EliminationTreeOrdered<DiscreteFactor> ETree
# ETree::shared_ptr eTree = ETree::Create(graph, structure)
# #eTree->print(">>>>>>>>>>> Elimination Tree <<<<<<<<<<<<<<<<<")

# # eliminate normally and check solution
# DiscreteBayesNet::shared_ptr bayesNet = eTree->eliminate(&EliminateDiscrete)
# #  bayesNet->print(">>>>>>>>>>>>>> Bayes Net <<<<<<<<<<<<<<<<<<")
# DiscreteFactor::sharedValues actualMPE = optimize(*bayesNet)
# EXPECT(assert_equal(expectedMPE, *actualMPE))

# # Approximate and check solution
# #  DiscreteBayesNet::shared_ptr approximateNet = eTree->approximate()
# #  approximateNet->print(">>>>>>>>>>>>>> Approximate Net <<<<<<<<<<<<<<<<<<")
# #  EXPECT(assert_equal(expectedMPE, *actualMPE))
# #endif
# }
# #ifdef OLD

# /* ************************************************************************* */
# /**
#  * Key type for discrete conditionals
#  * Includes name and cardinality
#  */
# class Key2 {
# private:
# std::string wff_
# size_t cardinality_
# public:
# /** Constructor, defaults to binary */
# Key2(const std::string& name, size_t cardinality = 2) :
# wff_(name), cardinality_(cardinality) {
# }
# const std::string& name() const {
#   return wff_
# }

# /** provide streaming */
# friend std::ostream& operator <<(std::ostream &os, const Key2 &key)
# }

# struct Factor2 {
# std::string wff_
# Factor2() :
# wff_("@") {
# }
# Factor2(const std::string& s) :
# wff_(s) {
# }
# Factor2(const Key2& key) :
# wff_(key.name()) {
# }

# friend std::ostream& operator <<(std::ostream &os, const Factor2 &f)
# friend Factor2 operator -(const Key2& key)
# }

# std::ostream& operator <<(std::ostream &os, const Factor2 &f) {
# os << f.wff_
# return os
# }

# /** negation */
# Factor2 operator -(const Key2& key) {
# return Factor2("-" + key.name())
# }

# /** OR */
# Factor2 operator ||(const Factor2 &factor1, const Factor2 &factor2) {
# return Factor2(std::string("(") + factor1.wff_ + " || " + factor2.wff_ + ")")
# }

# /** AND */
# Factor2 operator &&(const Factor2 &factor1, const Factor2 &factor2) {
# return Factor2(std::string("(") + factor1.wff_ + " && " + factor2.wff_ + ")")
# }

# /** implies */
# Factor2 operator >>(const Factor2 &factor1, const Factor2 &factor2) {
# return Factor2(std::string("(") + factor1.wff_ + " >> " + factor2.wff_ + ")")
# }

# struct Graph2: public std::list<Factor2> {

# /** Add a factor graph*/
# #  void operator +=(const Graph2& graph) {
# #    for(const Factor2& f: graph)
# #        push_back(f)
# #  }
# friend std::ostream& operator <<(std::ostream &os, const Graph2& graph)

# }

# /** Add a factor */
# #Graph2 operator +=(Graph2& graph, const Factor2& factor) {
# #  graph.push_back(factor)
# #  return graph
# #}
# std::ostream& operator <<(std::ostream &os, const Graph2& graph) {
# for(const Factor2& f: graph)
# os << f << endl
# return os
# }

# /* ************************************************************************* */
# TEST(DiscreteFactorGraph, Sugar)
# {
# Key2 M("Mythical"), I("Immortal"), A("Mammal"), H("Horned"), G("Magical")

# # Test this desired construction
# Graph2 unicorns
# unicorns += M >> -A
# unicorns += (-M) >> (-I && A)
# unicorns += (I || A) >> H
# unicorns += H >> G

# # should be done by adapting boost::assign:
# # unicorns += (-M) >> (-I && A), (I || A) >> H , H >> G

# cout << unicorns
# }
# #endif

# /* ************************************************************************* */
# int main() {
# TestResult tr
# return TestRegistry::runAllTests(tr)
# }
# /* ************************************************************************* */

