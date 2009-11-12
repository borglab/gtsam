/**
 *  @file   testGaussianConditional.cpp
 *  @brief  Unit tests for Conditional gaussian
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#include "Matrix.h"
#include "GaussianConditional.h"

using namespace gtsam;

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
    expected("x",d, R, "x1", A1, "l1", A2, tau),
    actual("x",d, R, "x1", A1, "l1", A2, tau);
  
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

  GaussianConditional cg("x",d, R, "x1", A1, "l1", A2, tau);
  
  Vector sx1(2);
  sx1(0) = 1.0; sx1(1) = 1.0;
  
  Vector sl1(2);
  sl1(0) = 1.0; sl1(1) = 1.0;
  
  VectorConfig solution;
  solution.insert("x1", sx1);
  solution.insert("l1", sl1);
  
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

	 GaussianConditional cg("x2", d, R, "x1", A1, "l1", A2);

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
