/**
 * @file    testGaussianBayesNet.cpp
 * @brief   Unit tests for GaussianBayesNet
 * @author  Frank Dellaert
 */

// STL/C++
#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#include "GaussianBayesNet.h"
#include "BayesNet.h"
#include "smallExample.h"
#include "Ordering.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( GaussianBayesNet, constructor )
{
  // small Bayes Net x <- y
  // x y d
  // 1 1 9
  //   1 5
  Matrix R11 = Matrix_(1,1,1.0), S12 = Matrix_(1,1,1.0);
  Matrix                         R22 = Matrix_(1,1,1.0);
  Vector d1(1), d2(1);
  d1(0) = 9; d2(0) = 5;
  Vector sigmas(1);
  sigmas(0) = 1.;
  
  // define nodes and specify in reverse topological sort (i.e. parents last)
  GaussianConditional x("x",d1,R11,"y",S12, sigmas), y("y",d2,R22, sigmas);

  // check small example which uses constructor
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  CHECK( x.equals(*cbn["x"]) );
  CHECK( y.equals(*cbn["y"]) );
}

/* ************************************************************************* */
TEST( GaussianBayesNet, matrix )
{
  // Create a test graph
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  Matrix R; Vector d;
  boost::tie(R,d) = matrix(cbn); // find matrix and RHS

  Matrix R1 = Matrix_(2,2,
		      1.0, 1.0,
		      0.0, 1.0
    );
  Vector d1 = Vector_(2, 9.0, 5.0);

  EQUALITY(R,R1);
  CHECK(d==d1);
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize )
{
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  VectorConfig actual = optimize(cbn);

  VectorConfig expected;
  Vector x(1), y(1);
  x(0) = 4; y(0) = 5;
  expected.insert("x",x);
  expected.insert("y",y);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( GaussianBayesNet, serialize )
{
	//create a starting CBN
	GaussianBayesNet cbn = createSmallGaussianBayesNet();

	//serialize the CBN
	ostringstream in_archive_stream;
	boost::archive::text_oarchive in_archive(in_archive_stream);
	in_archive << cbn;
	string serialized = in_archive_stream.str();

	//DEBUG
	cout << "CBN Raw string: [" << serialized << "]" << endl;

	//remove newlines/carriage returns
	string clean;
	BOOST_FOREACH(char s, serialized) {
		if (s != '\n') {
			//copy in character
			clean.append(string(1,s));
			}
		else {
			cout << "   Newline character found!" << endl;
			//replace with an identifiable string
			clean.append(string(1,' '));
			}
		}

	cout << "Cleaned CBN String: [" << clean << "]" << endl;

	//deserialize the CBN
	istringstream out_archive_stream(clean);
	boost::archive::text_iarchive out_archive(out_archive_stream);
	GaussianBayesNet output;
	out_archive >> output;
	CHECK(cbn.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
