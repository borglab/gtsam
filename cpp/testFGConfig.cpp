/**
 * @file   testFGConfig.cpp
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
#include "FGConfig.h"
#include "smallExample.cpp"

using namespace std;
using namespace gtsam;
	
/* ************************************************************************* */
TEST( FGConfig, equals )
 {	 
   FGConfig expected;
   Vector v = Vector_(3, 5.0, 6.0, 7.0);
   expected.insert("a",v);
   FGConfig actual;
   actual.insert("a",v);
   CHECK(actual.equals(expected));
 }

/* ************************************************************************* */
TEST( FGConfig, contains)
{
  FGConfig fg;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  fg.insert("ali", v);
  CHECK(fg.contains("ali"));
  CHECK(!fg.contains("gholi"));
}

/* ************************************************************************* */
TEST( FGConfig, plus)
{
  FGConfig fg;
  Vector vx = Vector_(3, 5.0, 6.0, 7.0), vy = Vector_(2, 8.0, 9.0);
  fg.insert("x", vx).insert("y",vy);

  FGConfig delta;
  Vector dx = Vector_(3, 1.0, 1.0, 1.0), dy = Vector_(2, -1.0, -1.0);
  delta.insert("x", dx).insert("y",dy);

  FGConfig expected;
  Vector wx = Vector_(3, 6.0, 7.0, 8.0), wy = Vector_(2, 7.0, 8.0);
  expected.insert("x", wx).insert("y",wy);

  // functional
  FGConfig actual = fg + delta;
  CHECK(actual.equals(expected));

  // inplace
  fg += delta;
  CHECK(fg.equals(expected));
}

/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( FGConfig, serialize)
{
    //DEBUG:
    cout << "FGConfig: Running Serialization Test" << endl;
    
    //create an FGConfig
    FGConfig fg = createConfig();
    
    //serialize the config
    std::ostringstream in_archive_stream;
    boost::archive::text_oarchive in_archive(in_archive_stream);
    in_archive << fg;
    std::string serialized_fgc = in_archive_stream.str();
    
    //deserialize the config
    std::istringstream out_archive_stream(serialized_fgc);
    boost::archive::text_iarchive out_archive(out_archive_stream);
    FGConfig output;
    out_archive >> output;
    
    //check for equality
    CHECK(fg.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
