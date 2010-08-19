/**
 * @file    testBinaryBayesNet.cpp
 * @brief   Unit tests for BinaryBayesNet
 * @author  Manohar Paluri
 */

// STL/C++
#include <iostream>
#include <sstream>
#include <map>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/BinaryConditional.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/SymbolMap.h>

using namespace std;
using namespace gtsam;

/** A Bayes net made from binary conditional probability tables */
typedef BayesNet<BinaryConditional> BinaryBayesNet;


double probability( BinaryBayesNet & bbn, SymbolMap<bool> & config)
{
	double result = 1.0;
	BinaryBayesNet::const_iterator it = bbn.begin();
	while( it != bbn.end() ){
		result *= (*it)->probability(config);
		it++;
	}
	return result;
}

/************************************************************************** */
TEST( BinaryBayesNet, constructor )
{
	// small Bayes Net x <- y
	// p(y) = 0.2
	// p(x|y=0) = 0.3
	// p(x|y=1) = 0.6

	SymbolMap<bool> config;
	config["y"] = false;
	config["x"] = false;
	// unary conditional for y
	boost::shared_ptr<BinaryConditional> py(new BinaryConditional("y",0.2));
	DOUBLES_EQUAL(0.8,py->probability(config),0.01);

	// single parent conditional for x
	vector<double> cpt;
	cpt += 0.3, 0.6 ; // array index corresponds to binary parent configuration
	boost::shared_ptr<BinaryConditional> px_y(new BinaryConditional("x","y",cpt));
	DOUBLES_EQUAL(0.7,px_y->probability(config),0.01);

	// push back conditionals in topological sort order (parents last)
	BinaryBayesNet bbn;
	bbn.push_back(py);
	bbn.push_back(px_y);

	// Test probability of 00,01,10,11
	DOUBLES_EQUAL(0.56,probability(bbn,config),0.01); // P(y=0)P(x=0|y=0) = 0.8 * 0.7 = 0.56;
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
