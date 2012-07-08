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
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem.hpp>
#include <CppUnitLite/TestHarness.h>

#include <wrap/utilities.h>
#include <wrap/Module.h>

using namespace std;
using namespace boost::assign;
using namespace wrap;
namespace fs = boost::filesystem;
static bool enable_verbose = false;
#ifdef TOPSRCDIR
static string topdir = TOPSRCDIR;
#else
static string topdir = "TOPSRCDIR_NOT_CONFIGURED"; // If TOPSRCDIR is not defined, we error
#endif

typedef vector<string> strvec;

// NOTE: as this path is only used to generate makefiles, it is hardcoded here for testing
// In practice, this path will be an absolute system path, which makes testing it more annoying
static const std::string headerPath = "/not_really_a_real_path/borg/gtsam/wrap";

/* ************************************************************************* */
TEST( wrap, ArgumentList ) {
	ArgumentList args;
	Argument arg1; arg1.type = "double"; arg1.name = "x";
	Argument arg2; arg2.type = "double"; arg2.name = "y";
	Argument arg3; arg3.type = "double"; arg3.name = "z";
	args.push_back(arg1);
	args.push_back(arg2);
	args.push_back(arg3);
	EXPECT(assert_equal("ddd", args.signature()));
	EXPECT(assert_equal("double,double,double", args.types()));
	EXPECT(assert_equal("x,y,z", args.names()));
}

/* ************************************************************************* */
TEST( wrap, check_exception ) {
	THROWS_EXCEPTION(Module("/notarealpath", "geometry",enable_verbose));
	CHECK_EXCEPTION(Module("/alsonotarealpath", "geometry",enable_verbose), CantOpenFile);

	// clean out previous generated code
	fs::remove_all("actual_deps");

	string path = topdir + "/wrap/tests";
	Module module(path.c_str(), "testDependencies",enable_verbose);
	CHECK_EXCEPTION(module.matlab_code("actual_deps", headerPath), DependencyMissing);
}

/* ************************************************************************* */
TEST( wrap, parse ) {
	string markup_header_path = topdir + "/wrap/tests";
	Module module(markup_header_path.c_str(), "geometry",enable_verbose);
	EXPECT_LONGS_EQUAL(3, module.classes.size());

	// check using declarations
	strvec exp_using1, exp_using2; exp_using2 += "geometry";

	// forward declarations
	strvec exp_forward; exp_forward += "VectorNotEigen", "ns::OtherClass";
	EXPECT(assert_equal(exp_forward, module.forward_declarations));

	// check first class, Point2
	{
		Class cls = module.classes.at(0);
		EXPECT(assert_equal("Point2", cls.name));
		EXPECT_LONGS_EQUAL(2, cls.constructor.args_list.size());
		EXPECT_LONGS_EQUAL(7, cls.methods.size());
		EXPECT_LONGS_EQUAL(0, cls.static_methods.size());
		EXPECT_LONGS_EQUAL(0, cls.namespaces.size());
		EXPECT(assert_equal(exp_using1, cls.using_namespaces));
	}

	// check second class, Point3
	{
		Class cls = module.classes.at(1);
		EXPECT(assert_equal("Point3", cls.name));
		EXPECT_LONGS_EQUAL(1, cls.constructor.args_list.size());
		EXPECT_LONGS_EQUAL(1, cls.methods.size());
		EXPECT_LONGS_EQUAL(2, cls.static_methods.size());
		EXPECT_LONGS_EQUAL(0, cls.namespaces.size());
		EXPECT(assert_equal(exp_using2, cls.using_namespaces));

		// first constructor takes 3 doubles
		ArgumentList c1 = cls.constructor.args_list.front();
		EXPECT_LONGS_EQUAL(3, c1.size());

		// check first double argument
		Argument a1 = c1.front();
		EXPECT(!a1.is_const);
		EXPECT(assert_equal("double", a1.type));
		EXPECT(!a1.is_ref);
		EXPECT(assert_equal("x", a1.name));

		// check method
		CHECK(cls.methods.find("norm") != cls.methods.end());
		Method m1 = cls.methods.find("norm")->second;
		LONGS_EQUAL(1, m1.returnVals.size());
		EXPECT(assert_equal("double", m1.returnVals.front().type1));
		EXPECT(assert_equal("norm", m1.name));
		LONGS_EQUAL(1, m1.argLists.size());
		EXPECT_LONGS_EQUAL(0, m1.argLists.front().size());
		EXPECT(m1.is_const_);
	}

	// Test class is the third one
	{
		LONGS_EQUAL(3, module.classes.size());
		Class testCls = module.classes.at(2);
		EXPECT_LONGS_EQUAL( 2, testCls.constructor.args_list.size());
		EXPECT_LONGS_EQUAL(19, testCls.methods.size());
		EXPECT_LONGS_EQUAL( 0, testCls.static_methods.size());
		EXPECT_LONGS_EQUAL( 0, testCls.namespaces.size());
		EXPECT(assert_equal(exp_using2, testCls.using_namespaces));
		strvec exp_includes; exp_includes += "folder/path/to/Test.h";
		EXPECT(assert_equal(exp_includes, testCls.includes));

		// function to parse: pair<Vector,Matrix> return_pair (Vector v, Matrix A) const;
		CHECK(testCls.methods.find("return_pair") != testCls.methods.end());
		Method m2 = testCls.methods.find("return_pair")->second;
		LONGS_EQUAL(1, m2.returnVals.size());
		EXPECT(m2.returnVals.front().isPair);
		EXPECT(m2.returnVals.front().category1 == ReturnValue::EIGEN);
		EXPECT(m2.returnVals.front().category2 == ReturnValue::EIGEN);
	}
}

/* ************************************************************************* */
TEST( wrap, parse_namespaces ) {
	string header_path = topdir + "/wrap/tests";
	Module module(header_path.c_str(), "testNamespaces",enable_verbose);
	EXPECT_LONGS_EQUAL(6, module.classes.size());

	{
		Class cls = module.classes.at(0);
		EXPECT(assert_equal("ClassA", cls.name));
		strvec exp_namespaces; exp_namespaces += "ns1";
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "path/to/ns1.h", "";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}

	{
		Class cls = module.classes.at(1);
		EXPECT(assert_equal("ClassB", cls.name));
		strvec exp_namespaces; exp_namespaces += "ns1";
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "path/to/ns1.h", "path/to/ns1/ClassB.h";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}

	{
		Class cls = module.classes.at(2);
		EXPECT(assert_equal("ClassA", cls.name));
		strvec exp_namespaces; exp_namespaces += "ns2";
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "path/to/ns2.h", "path/to/ns2/ClassA.h";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}

	{
		Class cls = module.classes.at(3);
		EXPECT(assert_equal("ClassB", cls.name));
		strvec exp_namespaces; exp_namespaces += "ns2", "ns3";
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "path/to/ns2.h", "path/to/ns3.h", "";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}

	{
		Class cls = module.classes.at(4);
		EXPECT(assert_equal("ClassC", cls.name));
		strvec exp_namespaces; exp_namespaces += "ns2";
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "path/to/ns2.h", "";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}

	{
		Class cls = module.classes.at(5);
		EXPECT(assert_equal("ClassD", cls.name));
		strvec exp_namespaces;
		EXPECT(assert_equal(exp_namespaces, cls.namespaces));
		strvec exp_includes; exp_includes += "";
		EXPECT(assert_equal(exp_includes, cls.includes));
	}
}

/* ************************************************************************* */
TEST( wrap, matlab_code_namespaces ) {
	string header_path = topdir + "/wrap/tests";
	Module module(header_path.c_str(), "testNamespaces",enable_verbose);
	EXPECT_LONGS_EQUAL(6, module.classes.size());
	string path = topdir + "/wrap";

	// clean out previous generated code
	fs::remove_all("actual_namespaces");

	// emit MATLAB code
	string exp_path = path + "/tests/expected_namespaces/";
	string act_path = "actual_namespaces/";
	module.matlab_code("actual_namespaces", headerPath);


	EXPECT(files_equal(exp_path + "ClassD.m"                    , act_path + "ClassD.m"                   ));
	EXPECT(files_equal(exp_path + "ns1ClassA.m"                 , act_path + "ns1ClassA.m"                ));
	EXPECT(files_equal(exp_path + "ns1ClassB.m"                 , act_path + "ns1ClassB.m"                ));
	EXPECT(files_equal(exp_path + "ns2ClassA.m"                 , act_path + "ns2ClassA.m"                ));
	EXPECT(files_equal(exp_path + "ns2ClassC.m"                 , act_path + "ns2ClassC.m"                ));
	EXPECT(files_equal(exp_path + "ns2ns3ClassB.m"              , act_path + "ns2ns3ClassB.m"             ));
	EXPECT(files_equal(exp_path + "testNamespaces_wrapper.cpp"  , act_path + "testNamespaces_wrapper.cpp" ));
}

/* ************************************************************************* */
TEST( wrap, matlab_code ) {
	// Parse into class object
	string header_path = topdir + "/wrap/tests";
	Module module(header_path,"geometry",enable_verbose);
	string path = topdir + "/wrap";

	// clean out previous generated code
	fs::remove_all("actual");

	// emit MATLAB code
	// make_geometry will not compile, use make testwrap to generate real make
	module.matlab_code("actual", headerPath);
	string epath = path + "/tests/expected/";
	string apath = "actual/";

	EXPECT(files_equal(epath + "geometry_wrapper.cpp" , apath + "geometry_wrapper.cpp" ));
	EXPECT(files_equal(epath + "Point2.m"             , apath + "Point2.m"             ));
	EXPECT(files_equal(epath + "Point3.m"             , apath + "Point3.m"             ));
	EXPECT(files_equal(epath + "Test.m"               , apath + "Test.m"               ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
