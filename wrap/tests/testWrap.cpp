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

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>

#include <wrap/utilities.h>
#include <wrap/Module.h>

using namespace std;
using namespace wrap;
static bool enable_verbose = false;
#ifdef TOPSRCDIR
static string topdir = TOPSRCDIR;
#else
static string topdir = "TOPSRCDIR_NOT_CONFIGURED"; // If TOPSRCDIR is not defined, we error
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
	THROWS_EXCEPTION(Module("/notarealpath", "geometry",enable_verbose));
	CHECK_EXCEPTION(Module("/alsonotarealpath", "geometry",enable_verbose), CantOpenFile);

	// clean out previous generated code
  string cleanCmd = "rm -rf actual";
  system(cleanCmd.c_str());

	string path = topdir + "/wrap/tests";
	Module module(path.c_str(), "testWrap1",enable_verbose);
	CHECK_EXCEPTION(module.matlab_code("actual", "", "mexa64", "-O5"), DependencyMissing);
}

/* ************************************************************************* */
TEST( wrap, parse ) {
	string header_path = topdir + "/wrap/tests";
	Module module(header_path.c_str(), "geometry",enable_verbose);
	EXPECT_LONGS_EQUAL(3, module.classes.size());

	// check first class, Point2
	{
		Class cls = module.classes.at(0);
		EXPECT(assert_equal("Point2", cls.name));
		EXPECT_LONGS_EQUAL(2, cls.constructors.size());
		EXPECT_LONGS_EQUAL(4, cls.methods.size());
		EXPECT_LONGS_EQUAL(0, cls.static_methods.size());
		EXPECT_LONGS_EQUAL(0, cls.namespaces.size());
	}

	// check second class, Point3
	{
		Class cls = module.classes.at(1);
		EXPECT(assert_equal("Point3", cls.name));
		EXPECT_LONGS_EQUAL(1, cls.constructors.size());
		EXPECT_LONGS_EQUAL(1, cls.methods.size());
		EXPECT_LONGS_EQUAL(2, cls.static_methods.size());
		EXPECT_LONGS_EQUAL(0, cls.namespaces.size());

		// first constructor takes 3 doubles
		Constructor c1 = cls.constructors.front();
		EXPECT_LONGS_EQUAL(3, c1.args.size());

		// check first double argument
		Argument a1 = c1.args.front();
		EXPECT(!a1.is_const);
		EXPECT(assert_equal("double", a1.type));
		EXPECT(!a1.is_ref);
		EXPECT(assert_equal("x", a1.name));

		// check method
		Method m1 = cls.methods.front();
		EXPECT(assert_equal("double", m1.returnVal.type1));
		EXPECT(assert_equal("norm", m1.name));
		EXPECT_LONGS_EQUAL(0, m1.args.size());
		EXPECT(m1.is_const_);
	}

	// Test class is the third one
	{
		LONGS_EQUAL(3, module.classes.size());
		Class testCls = module.classes.at(2);
		EXPECT_LONGS_EQUAL( 2, testCls.constructors.size());
		EXPECT_LONGS_EQUAL(19, testCls.methods.size());
		EXPECT_LONGS_EQUAL( 0, testCls.static_methods.size());
		EXPECT_LONGS_EQUAL( 0, testCls.namespaces.size());
		EXPECT_LONGS_EQUAL( 1, testCls.includes.size());
		EXPECT(assert_equal("folder/path/to/Test.h", testCls.includes.front()));

		// function to parse: pair<Vector,Matrix> return_pair (Vector v, Matrix A) const;
		Method m2 = testCls.methods.front();
		EXPECT(m2.returnVal.isPair);
		EXPECT(m2.returnVal.category1 == ReturnValue::EIGEN);
		EXPECT(m2.returnVal.category2 == ReturnValue::EIGEN);
	}
}

/* ************************************************************************* */
TEST( wrap, parse_namespaces ) {
	string header_path = topdir + "/wrap/tests";
	Module module(header_path.c_str(), "testNamespaces",enable_verbose);
	EXPECT_LONGS_EQUAL(6, module.classes.size());

	Class cls1 = module.classes.at(0);
	EXPECT(assert_equal("ClassA", cls1.name));
	EXPECT_LONGS_EQUAL(1, cls1.namespaces.size());
	EXPECT(assert_equal("ns1", cls1.namespaces.front()));

	Class cls2 = module.classes.at(1);
	EXPECT(assert_equal("ClassB", cls2.name));
	EXPECT_LONGS_EQUAL(1, cls2.namespaces.size());
	EXPECT(assert_equal("ns1", cls2.namespaces.front()));

	Class cls3 = module.classes.at(2);
	EXPECT(assert_equal("ClassA", cls3.name));
	EXPECT_LONGS_EQUAL(1, cls3.namespaces.size());
	EXPECT(assert_equal("ns2", cls3.namespaces.front()));

	Class cls4 = module.classes.at(3);
	EXPECT(assert_equal("ClassB", cls4.name));
	EXPECT_LONGS_EQUAL(2, cls4.namespaces.size());
	EXPECT(assert_equal("ns2", cls4.namespaces.front()));
	EXPECT(assert_equal("ns3", cls4.namespaces.back()));

	Class cls5 = module.classes.at(4);
	EXPECT(assert_equal("ClassC", cls5.name));
	EXPECT_LONGS_EQUAL(1, cls5.namespaces.size());
	EXPECT(assert_equal("ns2", cls5.namespaces.front()));

	Class cls6 = module.classes.at(5);
	EXPECT(assert_equal("ClassD", cls6.name));
	EXPECT_LONGS_EQUAL(0, cls6.namespaces.size());
}

/* ************************************************************************* */
TEST( wrap, matlab_code_namespaces ) {
	string header_path = topdir + "/wrap/tests";
	Module module(header_path.c_str(), "testNamespaces",enable_verbose);
	EXPECT_LONGS_EQUAL(6, module.classes.size());
	string path = topdir + "/wrap";

	// clean out previous generated code
  string cleanCmd = "rm -rf actual_namespaces";
  system(cleanCmd.c_str());

	// emit MATLAB code
  string exp_path = path + "/tests/expected_namespaces/";
  string act_path = "actual_namespaces/";
	module.matlab_code("actual_namespaces", "", "mexa64", "-O5");

	EXPECT(files_equal(exp_path + "make_testNamespaces.m", act_path + "make_testNamespaces.m"));
	EXPECT(files_equal(exp_path + "Makefile"       , act_path + "Makefile"       ));
}

/* ************************************************************************* */
TEST( wrap, matlab_code ) {
	// Parse into class object
	string header_path = topdir + "/wrap/tests";
	Module module(header_path,"geometry",enable_verbose);
	string path = topdir + "/wrap";

	// clean out previous generated code
  string cleanCmd = "rm -rf actual";
  system(cleanCmd.c_str());

	// emit MATLAB code
	// make_geometry will not compile, use make testwrap to generate real make
	module.matlab_code("actual", "", "mexa64", "-O5");

	EXPECT(files_equal(path + "/tests/expected/@Point2/Point2.m"  , "actual/@Point2/Point2.m"  ));
	EXPECT(files_equal(path + "/tests/expected/@Point2/x.cpp"     , "actual/@Point2/x.cpp"     ));
	EXPECT(files_equal(path + "/tests/expected/@Point3/Point3.m"  , "actual/@Point3/Point3.m"  ));
	EXPECT(files_equal(path + "/tests/expected/new_Point3_ddd.m"  , "actual/new_Point3_ddd.m"  ));
	EXPECT(files_equal(path + "/tests/expected/new_Point3_ddd.cpp", "actual/new_Point3_ddd.cpp"));
	EXPECT(files_equal(path + "/tests/expected/Point3_staticFunction.m"  , "actual/Point3_staticFunction.m"  ));
	EXPECT(files_equal(path + "/tests/expected/Point3_staticFunction.cpp", "actual/Point3_staticFunction.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Point3/norm.m"    , "actual/@Point3/norm.m"    ));
	EXPECT(files_equal(path + "/tests/expected/@Point3/norm.cpp"  , "actual/@Point3/norm.cpp"  ));

	EXPECT(files_equal(path + "/tests/expected/new_Test_.cpp"           , "actual/new_Test_.cpp"           ));
	EXPECT(files_equal(path + "/tests/expected/@Test/Test.m"            , "actual/@Test/Test.m"            ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_string.cpp" , "actual/@Test/return_string.cpp" ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_pair.cpp"   , "actual/@Test/return_pair.cpp"   ));
	EXPECT(files_equal(path + "/tests/expected/@Test/create_MixedPtrs.cpp", "actual/@Test/create_MixedPtrs.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_field.cpp"  , "actual/@Test/return_field.cpp"  ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_TestPtr.cpp", "actual/@Test/return_TestPtr.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_Test.cpp"   , "actual/@Test/return_Test.cpp"   ));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_Point2Ptr.cpp", "actual/@Test/return_Point2Ptr.cpp"));
	EXPECT(files_equal(path + "/tests/expected/@Test/return_ptrs.cpp"   , "actual/@Test/return_ptrs.cpp"   ));
	EXPECT(files_equal(path + "/tests/expected/@Test/print.m"           , "actual/@Test/print.m"           ));
	EXPECT(files_equal(path + "/tests/expected/@Test/print.cpp"         , "actual/@Test/print.cpp"         ));

	EXPECT(files_equal(path + "/tests/expected/make_geometry.m", "actual/make_geometry.m"));
	EXPECT(files_equal(path + "/tests/expected/Makefile"       , "actual/Makefile"       ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
