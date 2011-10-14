/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testWrap.cpp
 * @brief Unit test for wrap.c
 * @author Frank Dellaert
 **/

#include <iostream>
#include <fstream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>

#include <wrap/utilities.h>
#include <wrap/Module.h>

using namespace std;
static bool verbose = false;
#ifdef TOPSRCDIR
static string topdir = TOPSRCDIR;
#else
static string topdir = "penguin";
#endif

/* ************************************************************************* */
TEST( wrap, ArgumentList ) {
  ArgumentList args;
  Argument arg; arg.type = "double"; arg.name = "x";
  args.push_back(arg);
  args.push_back(arg);
  args.push_back(arg);
  CHECK(args.signature()=="ddd");
  EXPECT(args.types()=="double,double,double");
  EXPECT(args.names()=="x,x,x");
}

/* ************************************************************************* */
TEST( wrap, check_exception ) {
	THROWS_EXCEPTION(Module("/notarealpath", "geometry",verbose));
	CHECK_EXCEPTION(Module("/alsonotarealpath", "geometry",verbose), CantOpenFile);
}

/* ************************************************************************* */
TEST( wrap, parse ) {
	string path = topdir + "/wrap";

	Module module(path.c_str(), "geometry",verbose);
	EXPECT(module.classes.size()==3);

	// check second class, Point3
	Class cls = *(++module.classes.begin());
	EXPECT(cls.name=="Point3");
	EXPECT(cls.constructors.size()==1);
	EXPECT(cls.methods.size()==1);

	// first constructor takes 3 doubles
	Constructor c1 = cls.constructors.front();
	EXPECT(c1.args.size()==3);

	// check first double argument
	Argument a1 = c1.args.front();
	EXPECT(!a1.is_const);
	EXPECT(a1.type=="double");
	EXPECT(!a1.is_ref);
	EXPECT(a1.name=="x");

	// check method
	Method m1 = cls.methods.front();
	EXPECT(m1.returns=="double");
	EXPECT(m1.name=="norm");
	EXPECT(m1.args.size()==0);
	EXPECT(m1.is_const);
}

/* ************************************************************************* */
TEST( wrap, matlab_code ) {
	// Parse into class object
	string path = topdir + "/wrap";
	Module module(path,"geometry",verbose);

	// emit MATLAB code
	// make_geometry will not compile, use make testwrap to generate real make
	module.matlab_code("actual", "", "-O5");

	EXPECT(files_equal(path + "/tests/expected/@Point2/Point2.m"  , "actual/@Point2/Point2.m"  ));
	EXPECT(files_equal(path + "/tests/expected/@Point2/x.cpp"     , "actual/@Point2/x.cpp"     ));
	EXPECT(files_equal(path + "/tests/expected/@Point3/Point3.m"  , "actual/@Point3/Point3.m"  ));
	EXPECT(files_equal(path + "/tests/expected/new_Point3_ddd.m"  , "actual/new_Point3_ddd.m"  ));
	EXPECT(files_equal(path + "/tests/expected/new_Point3_ddd.cpp", "actual/new_Point3_ddd.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Point3/norm.m"    , "actual/@Point3/norm.m"    ));
	EXPECT(files_equal(path + "/tests/expected/@Point3/norm.cpp"  , "actual/@Point3/norm.cpp"  ));

	EXPECT(files_equal(path + "/tests/expected/new_Test_.cpp"           , "actual/new_Test_.cpp"           ));
	EXPECT(files_equal(path + "/tests/expected/@Test/Test.m"            , "actual/@Test/Test.m"            ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_string.cpp" , "actual/@Test/return_string.cpp" ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_pair.cpp"   , "actual/@Test/return_pair.cpp"   ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_field.cpp"  , "actual/@Test/return_field.cpp"  ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_TestPtr.cpp", "actual/@Test/return_TestPtr.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_ptrs.cpp"   , "actual/@Test/return_ptrs.cpp"   ));
	EXPECT(files_equal(path + "/tests/expected/@Test/print.m"           , "actual/@Test/print.m"           ));
	EXPECT(files_equal(path + "/tests/expected/@Test/print.cpp"         , "actual/@Test/print.cpp"         ));

	EXPECT(files_equal(path + "/tests/expected/make_geometry.m"   , "actual/make_geometry.m"   ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
