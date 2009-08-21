/*
 * testConstrainedChordalBayesNet.cpp
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "ConstrainedChordalBayesNet.h"
#include "smallExample.h"

using namespace gtsam;
using namespace std;

TEST ( ConstrainedChordalBayesNet, basic )
{
	ConstrainedChordalBayesNet ccbn = createConstrainedChordalBayesNet();
	FGConfig c = createConstrainedConfig();

	// get data back out
	DeltaFunction::shared_ptr x0 = ccbn.get_delta("x0");
	ConditionalGaussian::shared_ptr x1 = ccbn.get("x1");

	Matrix R = eye(2);
	Vector d = c["x1"];
	double sigma = 0.1;
	ConditionalGaussian::shared_ptr f1(new ConditionalGaussian(d/sigma, R/sigma));

	DeltaFunction::shared_ptr f2(new DeltaFunction(c["x0"], "x0"));

	CHECK(f1->equals(*x1));
	CHECK(f2->equals(*x0));
}

TEST ( ConstrainedChordalBayesNet, equals )
{
	// basic check
	ConstrainedChordalBayesNet ccbn1 = createConstrainedChordalBayesNet();
	ConstrainedChordalBayesNet ccbn2 = createConstrainedChordalBayesNet();
	CHECK(ccbn1.equals(ccbn2));

	// ensure deltas are compared
	ConstrainedChordalBayesNet ccbn3;
	FGConfig c = createConstrainedConfig();
	Matrix R = eye(2);
	Vector d = c["x1"];
	double sigma = 0.1;
	ConditionalGaussian::shared_ptr f1(new ConditionalGaussian(d/sigma, R/sigma));
	ccbn3.insert("x1", f1);

	CHECK(!ccbn1.equals(ccbn3));
}

TEST ( ConstrainedChordalBayesNet, copy )
{
	// use copy to allow testing with old example
	ChordalBayesNet cbn = createSmallChordalBayesNet();
	ConstrainedChordalBayesNet actual(cbn);

	ConditionalGaussian::shared_ptr x = cbn.get("x");
	ConditionalGaussian::shared_ptr y = cbn.get("y");

	ConstrainedChordalBayesNet expected;
	expected.insert("x",x);
	expected.insert("y",y);

	CHECK(assert_equal(actual, expected));
}

TEST ( ConstrainedChordalBayesNet, optimize_baseline )
{
	// optimize simple example
	ChordalBayesNet cbn = createSmallChordalBayesNet();
	ConstrainedChordalBayesNet ccbn(cbn);
	boost::shared_ptr<FGConfig> actual = ccbn.optimize();

	// create expected
	FGConfig expected;
	Vector x(1), y(1); x(0)=4.; y(0)=5.;
	expected.insert("x", x);
	expected.insert("y", y);

	// verify
	CHECK(expected.equals(*actual));
}

TEST ( ConstrainedChordalBayesNet, optimize )
{
	ConstrainedChordalBayesNet ccbn = createConstrainedChordalBayesNet();
	FGConfig expected = createConstrainedConfig();

	// full optimization
	boost::shared_ptr<FGConfig> actual1 = ccbn.optimize();
	CHECK(expected.equals(*actual1));

	// plug in a config
	boost::shared_ptr<FGConfig> c1(new FGConfig);
	c1->insert("x0", expected["x0"]);
	boost::shared_ptr<FGConfig> actual2 = ccbn.optimize(c1);
	CHECK(expected.equals(*actual2));

	// plug in the other value
	boost::shared_ptr<FGConfig> c2(new FGConfig);
	c2->insert("x1", expected["x1"]);
	boost::shared_ptr<FGConfig> actual3 = ccbn.optimize(c2);
	CHECK(expected.equals(*actual3));
}



/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

