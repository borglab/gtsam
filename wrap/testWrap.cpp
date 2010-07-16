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
static bool verbose = false;

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
TEST( wrap, check_exception ) {
	THROWS_EXCEPTION(Module(".", "geometry",verbose));
}

/* ************************************************************************* */
TEST( wrap, parse ) {
	Module module(".", "geometry",verbose);
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
	Module module(".","geometry",verbose);

	// emit MATLAB code
	// make_geometry will not compile, use make testwrap to generate real make
	module.matlab_code("actual", "", "-O5");

	CHECK(files_equal("expected/@Point2/Point2.m"  , "actual/@Point2/Point2.m"  ));
	CHECK(files_equal("expected/@Point2/x.cpp"     , "actual/@Point2/x.cpp"     ));

	CHECK(files_equal("expected/@Point3/Point3.m"  , "actual/@Point3/Point3.m"  ));
	CHECK(files_equal("expected/new_Point3_ddd.m"  , "actual/new_Point3_ddd.m"  ));
	CHECK(files_equal("expected/new_Point3_ddd.cpp", "actual/new_Point3_ddd.cpp"));
	CHECK(files_equal("expected/@Point3/norm.m"    , "actual/@Point3/norm.m"    ));
	CHECK(files_equal("expected/@Point3/norm.cpp"  , "actual/@Point3/norm.cpp"  ));

	CHECK(files_equal("expected/new_Test_.cpp"           , "actual/new_Test_.cpp"           ));
	CHECK(files_equal("expected/@Test/Test.m"            , "actual/@Test/Test.m"            ));
	CHECK(files_equal("expected/@Test/return_string.cpp" , "actual/@Test/return_string.cpp" ));
	CHECK(files_equal("expected/@Test/return_pair.cpp"   , "actual/@Test/return_pair.cpp"   ));
	CHECK(files_equal("expected/@Test/return_field.cpp"  , "actual/@Test/return_field.cpp"  ));
	CHECK(files_equal("expected/@Test/return_TestPtr.cpp", "actual/@Test/return_TestPtr.cpp"));
	CHECK(files_equal("expected/@Test/return_ptrs.cpp"   , "actual/@Test/return_ptrs.cpp"   ));
	CHECK(files_equal("expected/@Test/print.m"           , "actual/@Test/print.m"           ));
	CHECK(files_equal("expected/@Test/print.cpp"         , "actual/@Test/print.cpp"         ));

	CHECK(files_equal("expected/make_geometry.m"   , "actual/make_geometry.m"   ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
