/**
 * Unit test for wrap.c
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>

#include "utilities.h"
#include "Module.h"

using namespace std;

/* ************************************************************************* */
TEST( wrap, ArgumentList ) {
  ArgumentList args;
  Argument arg; arg.type = "double"; arg.name = "x";
  args.push_back(arg);
  args.push_back(arg);
  args.push_back(arg);
  CHECK(args.signature()=="ddd");
  CHECK(args.types()=="double,double,double");
  CHECK(args.names()=="x,x,x");
}

/* ************************************************************************* */
TEST( wrap, parse ) {
  Module module("../interfaces", "geometry");
  CHECK(module.classes.size()==3);

  // check second class, Point3
  Class cls = *(++module.classes.begin());
  CHECK(cls.name=="Point3");
  CHECK(cls.constructors.size()==1);
  CHECK(cls.methods.size()==1);

  // first constructor takes 3 doubles
  Constructor c1 = cls.constructors.front();
  CHECK(c1.args.size()==3);

  // check first double argument
  Argument a1 = c1.args.front();
  CHECK(!a1.is_const);
  CHECK(a1.type=="double");
  CHECK(!a1.is_ref);
  CHECK(a1.name=="x");

  // check method
  Method m1 = cls.methods.front();
  CHECK(m1.returns=="double");
  CHECK(m1.name=="norm");
  CHECK(m1.args.size()==0);
  CHECK(m1.is_const);
}

/* ************************************************************************* */
TEST( wrap, matlab_code ) {
  // Parse into class object
  Module module("../interfaces","geometry");

  // emit MATLAB code
  // make_geometry will not compile, use make testwrap to generate real make
  module.matlab_code("/Users/dellaert/projects/gtsam/matlab", "", "-O5");

  CHECK(files_equal("../matlab/@Point2/Point2.m"  , "../matlab/@Point2/Point2-expected.m"  ));
  CHECK(files_equal("../matlab/@Point2/x.cpp"     , "../matlab/@Point2/x-expected.cpp"     ));

  CHECK(files_equal("../matlab/@Point3/Point3.m"  , "../matlab/@Point3/Point3-expected.m"  ));
  CHECK(files_equal("../matlab/new_Point3_ddd.m"  , "../matlab/new_Point3_ddd-expected.m"  ));
  CHECK(files_equal("../matlab/new_Point3_ddd.cpp", "../matlab/new_Point3_ddd-expected.cpp"));
  CHECK(files_equal("../matlab/@Point3/norm.m"    , "../matlab/@Point3/norm-expected.m"    ));
  CHECK(files_equal("../matlab/@Point3/norm.cpp"  , "../matlab/@Point3/norm-expected.cpp"  ));

  CHECK(files_equal("../matlab/new_Test_.cpp"           , "../matlab/new_Test_-expected.cpp"           ));
  CHECK(files_equal("../matlab/@Test/Test.m"            , "../matlab/@Test/Test-expected.m"            ));
  CHECK(files_equal("../matlab/@Test/return_string.cpp" , "../matlab/@Test/return_string-expected.cpp" ));
  CHECK(files_equal("../matlab/@Test/return_pair.cpp"   , "../matlab/@Test/return_pair-expected.cpp"   ));
  CHECK(files_equal("../matlab/@Test/return_field.cpp"  , "../matlab/@Test/return_field-expected.cpp"  ));
  CHECK(files_equal("../matlab/@Test/return_TestPtr.cpp", "../matlab/@Test/return_TestPtr-expected.cpp"));
  CHECK(files_equal("../matlab/@Test/return_ptrs.cpp"   , "../matlab/@Test/return_ptrs-expected.cpp"   ));
  CHECK(files_equal("../matlab/@Test/print.m"           , "../matlab/@Test/print-expected.m"           ));
  CHECK(files_equal("../matlab/@Test/print.cpp"         , "../matlab/@Test/print-expected.cpp"         ));

  CHECK(files_equal("../matlab/make_geometry.m"   , "../matlab/make_geometry_expected.m"   ));
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0; }
/* ************************************************************************* */
