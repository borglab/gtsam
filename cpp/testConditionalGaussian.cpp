/**
 *  @file   testConditionalGaussian.cpp
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
#include "ConditionalGaussian.h"


using namespace gtsam;

/* ************************************************************************* */
/* untit test for equals                                                     */ 
/* ************************************************************************* */
TEST( ConditionalGaussian, equals )
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
  
  ConditionalGaussian 
    expected(d, R, "x1", A1, "l1", A2),
    actual(d, R, "x1", A1, "l1", A2);
  
  CHECK( expected.equals(actual) );
  
}

/* ************************************************************************* */
/* untit test for solve                                                      */ 
/* ************************************************************************* */
TEST( ConditionalGaussian, solve )
{
  
  //expected solution
  Vector expected(2);
  expected(0) = 15.0471 ; expected(1) = -18.8824;
  
  // create a conditional gaussion node
  Matrix A1 = Matrix_(2,2,  1., 2.,
                              3., 4.);
  
  Matrix A2 = Matrix_(2,2,  6., 0.2,
                              8., 0.4);
  
  Matrix R = Matrix_(2,2,   0.1, 0.3,
                              0.0, 0.34);
  
  Vector d(2);
  d(0) = 0.2; d(1) = 0.5;
  
  ConditionalGaussian cg(d, R, "x1", A1, "l1", A2);
  
  Vector sx1(2);
  sx1(0) = 0.2; sx1(1) = 0.5;
  
  Vector sl1(2);
  sl1(0) = 0.5; sl1(1) = 0.8;
  
  VectorConfig solution;
  solution.insert("x1", sx1);
  solution.insert("l1", sl1);
  
  Vector result = cg.solve(solution);

  CHECK( equal_with_abs_tol(expected , result, 0.0001));
  
}

/* ************************************************************************* */
/* unit test for serialization                                               */ 
/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( ConditionalGaussian, serialize )
{
//     // create a conditional gaussion node
//     Matrix A1(2,2);
//     A1(0,0) = 1 ; A1(1,0) = 2;
//     A1(0,1) = 3 ; A1(1,1) = 4;
// 
//     Matrix A2(2,2);
//     A2(0,0) = 6 ; A2(1,0) = 0.2;
//     A2(0,1) = 8 ; A2(1,1) = 0.4;
// 
//     Matrix R(2,2);
//     R(0,0) = 0.1 ; R(1,0) = 0.3;
//     R(0,1) = 0.0 ; R(1,1) = 0.34;
// 
//     Vector d(2);
//     d(0) = 0.2; d(1) = 0.5;
// 
//     ConditionalGaussian cg(d, R, "x1", A1, "l1", A2);
//     
//     //serialize the CG
//     std::ostringstream in_archive_stream;
//     boost::archive::text_oarchive in_archive(in_archive_stream);
//     in_archive << cg;
//     std::string serialized = in_archive_stream.str();
//     
//     //deserialize the CGg
//     std::istringstream out_archive_stream(serialized);
//     boost::archive::text_iarchive out_archive(out_archive_stream);
//     ConditionalGaussian output;
//     out_archive >> output;
//     
//     //check for equality
//     CHECK(cg.equals(output));

}
#endif //HAVE_BOOST_SERIALIZATION
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
