/**
 * @file   testVectorConfig.cpp
 * @brief  Unit tests for Factor Graph Configuration
 * @author Carlos Nieto
 **/

/*STL/C++*/
#include <iostream>
#include <sstream>

//#include TEST_AC_DEFINE

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#include <CppUnitLite/TestHarness.h>
#include "Matrix.h"
#include "VectorConfig.h"
#include "smallExample.cpp"

using namespace std;
using namespace gtsam;
	
/* ************************************************************************* */
TEST( VectorConfig, equals1 )
 {
   VectorConfig expected;
   Vector v = Vector_(3, 5.0, 6.0, 7.0);
   expected.insert("a",v);
   VectorConfig actual;
   actual.insert("a",v);
   CHECK(actual.equals(expected));
}

/* ************************************************************************* */
TEST( VectorConfig, equals2 )
 {	 
   VectorConfig cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
   cfg1.insert("x", v1);
   cfg2.insert("x", v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorConfig, equals_nan )
 {
   VectorConfig cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, 0.0/0.0, 0.0/0.0, 0.0/0.0);
   cfg1.insert("x", v1);
   cfg2.insert("x", v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorConfig, contains)
{
  VectorConfig fg;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  fg.insert("ali", v);
  CHECK(fg.contains("ali"));
  CHECK(!fg.contains("gholi"));
}

/* ************************************************************************* */
TEST( VectorConfig, plus)
{
  VectorConfig fg;
  Vector vx = Vector_(3, 5.0, 6.0, 7.0), vy = Vector_(2, 8.0, 9.0);
  fg.insert("x", vx).insert("y",vy);

  VectorConfig delta;
  Vector dx = Vector_(3, 1.0, 1.0, 1.0), dy = Vector_(2, -1.0, -1.0);
  delta.insert("x", dx).insert("y",dy);

  VectorConfig expected;
  Vector wx = Vector_(3, 6.0, 7.0, 8.0), wy = Vector_(2, 7.0, 8.0);
  expected.insert("x", wx).insert("y",wy);

  // functional
  VectorConfig actual = fg.exmap(delta);
  CHECK(actual.equals(expected));
}

/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( VectorConfig, serialize)
{
    //DEBUG:
    cout << "VectorConfig: Running Serialization Test" << endl;
    
    //create an VectorConfig
    VectorConfig fg = createConfig();
    
    //serialize the config
    std::ostringstream in_archive_stream;
    boost::archive::text_oarchive in_archive(in_archive_stream);
    in_archive << fg;
    std::string serialized_fgc = in_archive_stream.str();
    
    //deserialize the config
    std::istringstream out_archive_stream(serialized_fgc);
    boost::archive::text_iarchive out_archive(out_archive_stream);
    VectorConfig output;
    out_archive >> output;
    
    //check for equality
    CHECK(fg.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
