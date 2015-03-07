/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteBayesNet.cpp
 *
 *  @date Feb 27, 2011
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteSequentialSolver.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/map.hpp>
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteBayesNet, Asia)
{
	DiscreteBayesNet asia;
//	DiscreteKey A("Asia"), S("Smoking"), T("Tuberculosis"), L("LungCancer"), B(
//			"Bronchitis"), E("Either"), X("XRay"), D("Dyspnoea");
	DiscreteKey A(0,2), S(4,2), T(3,2), L(6,2), B(7,2), E(5,2), X(2,2), D(1,2);

	// TODO: make a version that doesn't use the parser
	add_front(asia, A % "99/1");
	add_front(asia, S % "50/50");

	add_front(asia, T | A = "99/1 95/5");
	add_front(asia, L | S = "99/1 90/10");
	add_front(asia, B | S = "70/30 40/60");

	add_front(asia, (E | T, L) = "F T T T");

	add_front(asia, X | E = "95/5 2/98");
	// next lines are same as add_front(asia, (D | E, B) = "9/1 2/8 3/7 1/9");
	DiscreteConditional::shared_ptr actual =
			boost::make_shared<DiscreteConditional>((D | E, B) = "9/1 2/8 3/7 1/9");
	asia.push_front(actual);
	//	GTSAM_PRINT(asia);

	// Convert to factor graph
	DiscreteFactorGraph fg(asia);
	//	GTSAM_PRINT(fg);
	LONGS_EQUAL(3,fg.front()->size());
	Potentials::ADT expected(B & D & E, "0.9 0.3 0.1 0.7 0.2 0.1 0.8 0.9");
	CHECK(assert_equal(expected,(Potentials::ADT)*actual));

	// Create solver and eliminate
	DiscreteSequentialSolver solver(fg);
	DiscreteBayesNet::shared_ptr chordal = solver.eliminate();
	//	GTSAM_PRINT(*chordal);
	DiscreteConditional expected2(B % "11/9");
	CHECK(assert_equal(expected2,*chordal->back()));

	// solve
	DiscreteFactor::sharedValues actualMPE = optimize(*chordal);
	DiscreteFactor::Values expectedMPE;
	insert(expectedMPE)(A.first, 0)(D.first, 0)(X.first, 0)(T.first, 0)(S.first,
			0)(E.first, 0)(L.first, 0)(B.first, 0);
	EXPECT(assert_equal(expectedMPE, *actualMPE));

	// add evidence, we were in Asia and we have Dispnoea
	fg.add(A, "0 1");
	fg.add(D, "0 1");
//	fg.product().dot("fg");

	// solve again, now with evidence
	DiscreteSequentialSolver solver2(fg);
	DiscreteBayesNet::shared_ptr chordal2 = solver2.eliminate();
//	GTSAM_PRINT(*chordal2);
	DiscreteFactor::sharedValues actualMPE2 = optimize(*chordal2);
	DiscreteFactor::Values expectedMPE2;
	insert(expectedMPE2)(A.first, 1)(D.first, 1)(X.first, 0)(T.first, 0)(S.first,
			1)(E.first, 0)(L.first, 0)(B.first, 1);
	EXPECT(assert_equal(expectedMPE2, *actualMPE2));

	// now sample from it
	DiscreteFactor::Values expectedSample;
	SETDEBUG("DiscreteConditional::sample", false);
	insert(expectedSample)(A.first, 1)(D.first, 1)(X.first, 0)(T.first, 0)(
			S.first, 1)(E.first, 0)(L.first, 0)(B.first, 1);
	DiscreteFactor::sharedValues actualSample = sample(*chordal2);
	EXPECT(assert_equal(expectedSample, *actualSample));
}

/* ************************************************************************* */
TEST_UNSAFE(DiscreteBayesNet, Sugar)
{
	DiscreteKey T(0,2), L(1,2), E(2,2), D(3,2), C(8,3), S(7,2);

	DiscreteBayesNet bn;

	// test some mistakes
	//	add(bn, D);
	//	add(bn, D | E);
	//	add(bn, D | E = "blah");

	// try logic
	add(bn, (E | T, L) = "OR");
	add(bn, (E | T, L) = "AND");

	//	// try multivalued
	add(bn, C % "1/1/2");
	add(bn, C | S = "1/1/2 5/2/3");
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

