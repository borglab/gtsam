/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianConditional.cpp
 *  @brief  Unit tests for Conditional gaussian
 *  @author Christian Potthast
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

//#define GTSAM_MAGIC_KEY

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianConditional.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <boost/assign/std/list.hpp>

using namespace gtsam;
using namespace std;
using namespace boost::assign;

static const Index _x_=0, _x1_=1, _l1_=2;

/* ************************************************************************* */
TEST(GaussianConditional, constructor)
{
  Matrix R = Matrix_(2,2,
      -12.1244,  -5.1962,
            0.,   4.6904);
  Matrix S1 = Matrix_(2,2,
      -5.2786,  -8.6603,
      5.0254,   5.5432);
  Matrix S2 = Matrix_(2,2,
      -10.5573,  -5.9385,
      5.5737,   3.0153);
  Matrix S3 = Matrix_(2,2,
      -11.3820,  -7.2581,
      -3.0153,  -3.5635);

  Vector d = Vector_(2, 1.0, 2.0);
  Vector s = Vector_(2, 3.0, 4.0);

  list<pair<Index, Matrix> > terms;
  terms +=
      make_pair(3, S1),
      make_pair(5, S2),
      make_pair(7, S3);

  GaussianConditional actual(1, d, R, terms, s);

  GaussianConditional::const_iterator it = actual.beginFrontals();
  CHECK(assert_equal(Index(1), *it));
  CHECK(assert_equal(R, actual.get_R()));
  ++ it;
  CHECK(it == actual.endFrontals());

  it = actual.beginParents();
  CHECK(assert_equal(Index(3), *it));
  CHECK(assert_equal(S1, actual.get_S(it)));

  ++ it;
  CHECK(assert_equal(Index(5), *it));
  CHECK(assert_equal(S2, actual.get_S(it)));

  ++ it;
  CHECK(assert_equal(Index(7), *it));
  CHECK(assert_equal(S3, actual.get_S(it)));

  ++it;
  CHECK(it == actual.endParents());

  CHECK(assert_equal(d, actual.get_d()));
  CHECK(assert_equal(s, actual.get_sigmas()));
}

/* ************************************************************************* */
/* unit test for equals                                                     */
/* ************************************************************************* */
TEST( GaussianConditional, equals )
{
  // create a conditional gaussian node
  Matrix A1(2,2);
  A1(0,0) = 1 ; A1(1,0) = 2;
  A1(0,1) = 3 ; A1(1,1) = 4;
  
  Matrix A2(2,2);
  A2(0,0) = 6 ; A2(1,0) = 0.2;
  A2(0,1) = 8 ; A2(1,1) = 0.4;
  
  Matrix R(2,2);
  R(0,0) = 0.1 ; R(1,0) = 0.3;
  R(0,1) = 0.0 ; R(1,1) = 0.34;
  
  Vector tau(2);
  tau(0) = 1.0;
  tau(1) = 0.34;

  Vector d(2);
  d(0) = 0.2; d(1) = 0.5;
  
  GaussianConditional 
    expected(_x_,d, R, _x1_, A1, _l1_, A2, tau),
    actual(_x_,d, R, _x1_, A1, _l1_, A2, tau);
  
  CHECK( expected.equals(actual) );
  
}

/* ************************************************************************* */
/* unit test for solve                                                      */
/* ************************************************************************* */
TEST( GaussianConditional, solve )
{
  //expected solution
  Vector expected(2);
  expected(0) = 20-3-11 ; expected(1) = 40-7-15;
  
  // create a conditional gaussion node
  Matrix R = Matrix_(2,2,   1., 0.,
                            0., 1.);

  Matrix A1 = Matrix_(2,2,  1., 2.,
                            3., 4.);
  
  Matrix A2 = Matrix_(2,2,  5., 6.,
                            7., 8.);
  
  Vector d(2);
  d(0) = 20.0; d(1) = 40.0;
  
  Vector tau = ones(2);

  GaussianConditional cg(_x_,d, R, _x1_, A1, _l1_, A2, tau);
  
  Vector sx1(2);
  sx1(0) = 1.0; sx1(1) = 1.0;
  
  Vector sl1(2);
  sl1(0) = 1.0; sl1(1) = 1.0;
  
  VectorValues solution(vector<size_t>(3, 2));
  solution[_x1_] = sx1;
  solution[_l1_] = sl1;
  
  Vector result = cg.solve(solution);

  CHECK(assert_equal(expected , result, 0.0001));
  
}

/* ************************************************************************* */
/* unit test for serialization                                               */ 
/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( GaussianConditional, serialize )
{
	 // create a conditional gaussion node
	 Matrix A1(2,2);
	 A1(0,0) = 1 ; A1(1,0) = 2;
	 A1(0,1) = 3 ; A1(1,1) = 4;

	 Matrix A2(2,2);
	 A2(0,0) = 6 ; A2(1,0) = 0.2;
	 A2(0,1) = 8 ; A2(1,1) = 0.4;

	 Matrix R(2,2);
	 R(0,0) = 0.1 ; R(1,0) = 0.3;
	 R(0,1) = 0.0 ; R(1,1) = 0.34;

	 Vector d(2);
	 d(0) = 0.2; d(1) = 0.5;

	 GaussianConditional cg(_x2_, d, R, _x1_, A1, _l1_, A2);

	 //serialize the CG
	 std::ostringstream in_archive_stream;
	 boost::archive::text_oarchive in_archive(in_archive_stream);
	 in_archive << cg;
	 std::string serialized = in_archive_stream.str();

	 //deserialize the CGg
	 std::istringstream out_archive_stream(serialized);
	 boost::archive::text_iarchive out_archive(out_archive_stream);
	 GaussianConditional output;
	 out_archive >> output;

	 //check for equality
	 CHECK(cg.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
