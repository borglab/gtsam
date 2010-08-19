/**
 * @file    testGaussianBayesNet.cpp
 * @brief   Unit tests for GaussianBayesNet
 * @author  Frank Dellaert
 */

// STL/C++
#include <iostream>
#include <sstream>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#define GTSAM_MAGIC_KEY

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

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
  expected.insert("x",Vector_(1,4.));
  expected.insert("y",Vector_(1,5.));

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize2 )
{

	// Create empty graph
	GaussianFactorGraph fg;
	SharedDiagonal noise = noiseModel::Unit::Create(1);

	fg.add("y", eye(1), 2*ones(1), noise);

	fg.add("x", eye(1),"y", -eye(1), -ones(1), noise);

	fg.add("y", eye(1),"z", -eye(1), -ones(1), noise);

	fg.add("z", eye(1),"x", -eye(1), 2*ones(1), noise);

	Ordering ordering; ordering += "x", "y", "z";
	GaussianBayesNet cbn = fg.eliminate(ordering);
  VectorConfig actual = optimize(cbn);

  VectorConfig expected;
  expected.insert("x",Vector_(1,1.));
  expected.insert("y",Vector_(1,2.));
  expected.insert("z",Vector_(1,3.));

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstitute )
{
	// y=R*x, x=inv(R)*y
	// 2 = 1 1  -1
	// 3     1   3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  VectorConfig y, x;
  y.insert("x",Vector_(1,2.));
  y.insert("y",Vector_(1,3.));
  x.insert("x",Vector_(1,-1.));
  x.insert("y",Vector_(1, 3.));

  // test functional version
  VectorConfig actual = backSubstitute(cbn,y);
  CHECK(assert_equal(x,actual));

  // test imperative version
  backSubstituteInPlace(cbn,y);
  CHECK(assert_equal(x,y));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, rhs )
{
	// y=R*x, x=inv(R)*y
	// 2 = 1 1  -1
	// 3     1   3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
	VectorConfig expected = gtsam::optimize(cbn);
	VectorConfig d = rhs(cbn);
	VectorConfig actual = backSubstitute(cbn, d);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, rhs_with_sigmas )
{
	Matrix R11 = Matrix_(1, 1, 1.0), S12 = Matrix_(1, 1, 1.0);
	Matrix R22 = Matrix_(1, 1, 1.0);
	Vector d1(1), d2(1);
	d1(0) = 9;
	d2(0) = 5;
	Vector tau(1);
	tau(0) = 0.25;

	// define nodes and specify in reverse topological sort (i.e. parents last)
	GaussianConditional::shared_ptr Px_y(new GaussianConditional("x", d1, R11,
			"y", S12, tau)), Py(new GaussianConditional("y", d2, R22, tau));
	GaussianBayesNet cbn;
	cbn.push_back(Px_y);
	cbn.push_back(Py);

	VectorConfig expected = gtsam::optimize(cbn);
	VectorConfig d = rhs(cbn);
	VectorConfig actual = backSubstitute(cbn, d);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstituteTranspose )
{
	// x=R'*y, y=inv(R')*x
	// 2 = 1    2
	// 5   1 1  3
  GaussianBayesNet cbn = createSmallGaussianBayesNet();

  VectorConfig x, y;
  x.insert("x",Vector_(1,2.));
  x.insert("y",Vector_(1,5.));
  y.insert("x",Vector_(1,2.));
  y.insert("y",Vector_(1,3.));

  // test functional version
  VectorConfig actual = backSubstituteTranspose(cbn,x);
  CHECK(assert_equal(y,actual));
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
