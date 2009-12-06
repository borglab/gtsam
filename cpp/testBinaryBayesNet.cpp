/**
 * @file    testBinaryBayesNet.cpp
 * @brief   Unit tests for BinaryBayesNet
 * @author  Manohar Paluri
 */

// STL/C++
#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#include "BinaryConditional.h"
#include "BayesNet-inl.h"
#include "smallExample.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;

/** A Bayes net made from binary conditional probability tables */
typedef BayesNet<BinaryConditional> BinaryBayesNet;

struct BinaryConfig {
	bool px_;
	bool py_;

	BinaryConfig( bool px, bool py ):px_(px), py_(py){}
};

double probability(const BinaryBayesNet& bayesNet, const BinaryConfig& config) {
	double result = 1.0;
	/* TODO: using config multiply the probabilities */
	return result;
}

/* ************************************************************************* */
TEST( BinaryBayesNet, constructor )
{
  // small Bayes Net x <- y
  // p(y) = 0.2
  // p(x|y=0) = 0.3
  // p(x|y=1) = 0.5

  // unary conditional for y
  boost::shared_ptr<BinaryConditional> py(new BinaryConditional("y",0.2));
  py->print("py");

  // single parent conditional for x
  vector<double> cpt;
  cpt += 0.7, 0.5, 0.3, 0.5 ; // array index corresponds to binary parent configuration
  boost::shared_ptr<BinaryConditional> px_y(new BinaryConditional("x","y",cpt));
  px_y->print("px_y");

  // push back conditionals in topological sort order (parents last)
  BinaryBayesNet bbn;
  bbn.push_back(py);
  bbn.push_back(px_y);

  // Test probability of 00,01,10,11
  DOUBLES_EQUAL(0.56,probability(bbn,BinaryConfig(false,false)),0.01); // P(y=0)P(x=0|y=0) = 0.8 * 0.7 = 0.56;
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
