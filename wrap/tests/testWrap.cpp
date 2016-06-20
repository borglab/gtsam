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

#include <wrap/utilities.h>
#include <wrap/Module.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>
#include <boost/filesystem.hpp>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

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

/* ************************************************************************* */
TEST( wrap, ArgumentList ) {
  ArgumentList args;
  Argument arg1; arg1.type.name_ = "double"; arg1.name = "x";
  Argument arg2; arg2.type.name_ = "double"; arg2.name = "y";
  Argument arg3; arg3.type.name_ = "double"; arg3.name = "z";
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

//  // TODO: matlab_code does not throw this anymore, so check constructor
//  fs::remove_all("actual_deps"); // clean out previous generated code
//  string path = topdir + "/wrap/tests";
//  Module module(path.c_str(), "testDependencies",enable_verbose);
//  CHECK_EXCEPTION(module.matlab_code("actual_deps"), DependencyMissing);
}

/* ************************************************************************* */
TEST( wrap, Small ) {
  string moduleName("gtsam");
  Module module(moduleName, true);

  string markup(
      string("class Point2 {                \n") +
      string(" double x() const;            \n") +   // Method 1
      string(" Matrix returnMatrix() const;   \n") + // Method 2
      string(" Point2 returnPoint2() const; \n") +   // Method 3
      string(" static Vector returnVector(); \n") +  // Static Method 1
      string("};\n"));
  module.parseMarkup(markup);

  // check return types
  LONGS_EQUAL(1, module.classes.size());
  Class cls = module.classes.front();
  EXPECT(assert_equal("Point2", cls.name()));
  EXPECT(!cls.isVirtual);
  EXPECT(cls.namespaces().empty());
  LONGS_EQUAL(3, cls.nrMethods());
  LONGS_EQUAL(1, cls.static_methods.size());

  // Method 1
  Method m1 = cls.method("x");
  EXPECT(assert_equal("x", m1.name()));
  EXPECT(m1.isConst());
  LONGS_EQUAL(1, m1.nrOverloads());

  ReturnValue rv1 = m1.returnValue(0);
  EXPECT(!rv1.isPair);
  EXPECT(!rv1.type1.isPtr);
  EXPECT(assert_equal("double", rv1.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::BASIS, rv1.type1.category);

  // Method 2
  Method m2 = cls.method("returnMatrix");
  EXPECT(assert_equal("returnMatrix", m2.name()));
  EXPECT(m2.isConst());
  LONGS_EQUAL(1, m2.nrOverloads());

  ReturnValue rv2 = m2.returnValue(0);
  EXPECT(!rv2.isPair);
  EXPECT(!rv2.type1.isPtr);
  EXPECT(assert_equal("Matrix", rv2.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::EIGEN, rv2.type1.category);

  // Method 3
  Method m3 = cls.method("returnPoint2");
  EXPECT(assert_equal("returnPoint2", m3.name()));
  EXPECT(m3.isConst());
  LONGS_EQUAL(1, m3.nrOverloads());

  ReturnValue rv3 = m3.returnValue(0);
  EXPECT(!rv3.isPair);
  EXPECT(!rv3.type1.isPtr);
  EXPECT(assert_equal("Point2", rv3.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::CLASS, rv3.type1.category);

  // Static Method 1
  // static Vector returnVector();
  StaticMethod sm1 = cls.static_methods.at("returnVector");
  EXPECT(assert_equal("returnVector", sm1.name()));
  LONGS_EQUAL(1, sm1.nrOverloads());

  ReturnValue rv4 = sm1.returnValue(0);
  EXPECT(!rv4.isPair);
  EXPECT(!rv4.type1.isPtr);
  EXPECT(assert_equal("Vector", rv4.type1.name()));
  EXPECT_LONGS_EQUAL(ReturnType::EIGEN, rv4.type1.category);

}

/* ************************************************************************* */
TEST( wrap, Geometry ) {
  string markup_header_path = topdir + "/wrap/tests";
  Module module(markup_header_path.c_str(), "geometry",enable_verbose);
  EXPECT_LONGS_EQUAL(9, module.classes.size());

  // forward declarations
  LONGS_EQUAL(2, module.forward_declarations.size());
  EXPECT(assert_equal("VectorNotEigen", module.forward_declarations[0].name));
  EXPECT(assert_equal("ns::OtherClass", module.forward_declarations[1].name));

  // includes
  strvec exp_includes; exp_includes += "folder/path/to/Test.h";
  EXPECT(assert_equal(exp_includes, module.includes));

  LONGS_EQUAL(9, module.classes.size());

  // Key for ReturnType::return_category
//  CLASS = 1,
//  EIGEN = 2,
//  BASIS = 3,
//  VOID  = 4,

  {
    // check first class
    //  class Point2 {
    //   Point2();
    //   Point2(double x, double y);
    //   double x() const;
    //   double y() const;
    //   int dim() const;
    //   char returnChar() const;
    //   void argChar(char a) const;
    //   void argUChar(unsigned char a) const;
    //   VectorNotEigen vectorConfusion();
    //  };

    Class cls = module.classes.at(0);
    EXPECT(assert_equal("Point2", cls.name()));
    EXPECT_LONGS_EQUAL(2, cls.constructor.nrOverloads());
    EXPECT_LONGS_EQUAL(8, cls.nrMethods());

    {
      //   char returnChar() const;
      CHECK(cls.exists("returnChar"));
      Method m1 = cls.method("returnChar");
      LONGS_EQUAL(1, m1.nrOverloads());
      EXPECT(assert_equal("char", m1.returnValue(0).type1.name()));
      EXPECT_LONGS_EQUAL(ReturnType::BASIS, m1.returnValue(0).type1.category);
      EXPECT(!m1.returnValue(0).isPair);
      EXPECT(assert_equal("returnChar", m1.name()));
      LONGS_EQUAL(1, m1.nrOverloads());
      EXPECT_LONGS_EQUAL(0, m1.argumentList(0).size());
      EXPECT(m1.isConst());
    }

    {
      //   VectorNotEigen vectorConfusion();
      CHECK(cls.exists("vectorConfusion"));
      Method m1 = cls.method("vectorConfusion");
      LONGS_EQUAL(1, m1.nrOverloads());
      EXPECT(assert_equal("VectorNotEigen", m1.returnValue(0).type1.name()));
      EXPECT_LONGS_EQUAL(ReturnType::CLASS, m1.returnValue(0).type1.category);
      EXPECT(!m1.returnValue(0).isPair);
      EXPECT(assert_equal("vectorConfusion", m1.name()));
      LONGS_EQUAL(1, m1.nrOverloads());
      EXPECT_LONGS_EQUAL(0, m1.argumentList(0).size());
      EXPECT(!m1.isConst());
    }

    EXPECT_LONGS_EQUAL(0, cls.static_methods.size());
    EXPECT_LONGS_EQUAL(1, cls.namespaces().size());

#ifndef WRAP_DISABLE_SERIALIZE
    // check serialization flag
    EXPECT(cls.isSerializable);
    EXPECT(!cls.hasSerialization);
#endif
    }

  // check second class, Point3
  {
    Class cls = module.classes.at(1);
    EXPECT(assert_equal("Point3", cls.name()));
    EXPECT_LONGS_EQUAL(1, cls.constructor.nrOverloads());
    EXPECT_LONGS_EQUAL(1, cls.nrMethods());
    EXPECT_LONGS_EQUAL(2, cls.static_methods.size());
    EXPECT_LONGS_EQUAL(1, cls.namespaces().size());

    // first constructor takes 3 doubles
    ArgumentList c1 = cls.constructor.argumentList(0);
    EXPECT_LONGS_EQUAL(3, c1.size());

    // check first double argument
    Argument a1 = c1.front();
    EXPECT(!a1.is_const);
    EXPECT(assert_equal("double", a1.type.name_));
    EXPECT(!a1.is_ref);
    EXPECT(assert_equal("x", a1.name));

    // check method
    CHECK(cls.exists("norm"));
    Method m1 = cls.method("norm");
    LONGS_EQUAL(1, m1.nrOverloads());
    EXPECT(assert_equal("double", m1.returnValue(0).type1.name()));
    EXPECT_LONGS_EQUAL(ReturnType::BASIS, m1.returnValue(0).type1.category);
    EXPECT(assert_equal("norm", m1.name()));
    LONGS_EQUAL(1, m1.nrOverloads());
    EXPECT_LONGS_EQUAL(0, m1.argumentList(0).size());
    EXPECT(m1.isConst());

#ifndef WRAP_DISABLE_SERIALIZE
    // check serialization flag
    EXPECT(cls.isSerializable);
    EXPECT(cls.hasSerialization);
#endif
    }

  // Test class is the third one
  {
    Class testCls = module.classes.at(2);
    EXPECT_LONGS_EQUAL( 2, testCls.constructor.nrOverloads());
    EXPECT_LONGS_EQUAL(19, testCls.nrMethods());
    EXPECT_LONGS_EQUAL( 0, testCls.static_methods.size());
    EXPECT_LONGS_EQUAL( 0, testCls.namespaces().size());

    // function to parse: pair<Vector,Matrix> return_pair (Vector v, Matrix A) const;
    CHECK(testCls.exists("return_pair"));
    Method m2 = testCls.method("return_pair");
    LONGS_EQUAL(1, m2.nrOverloads());
    EXPECT(m2.returnValue(0).isPair);
    EXPECT_LONGS_EQUAL(ReturnType::EIGEN, m2.returnValue(0).type1.category);
    EXPECT(assert_equal("Vector", m2.returnValue(0).type1.name()));
    EXPECT_LONGS_EQUAL(ReturnType::EIGEN, m2.returnValue(0).type2.category);
    EXPECT(assert_equal("Matrix", m2.returnValue(0).type2.name()));

    // checking pointer args and return values
//    pair<Test*,Test*> return_ptrs (Test* p1, Test* p2) const;
    CHECK(testCls.exists("return_ptrs"));
    Method m3 = testCls.method("return_ptrs");
    LONGS_EQUAL(1, m3.nrOverloads());
    ArgumentList args = m3.argumentList(0);
    LONGS_EQUAL(2, args.size());

    Argument arg1 = args.at(0);
    EXPECT(arg1.is_ptr);
    EXPECT(!arg1.is_const);
    EXPECT(!arg1.is_ref);
    EXPECT(assert_equal("Test", arg1.type.name_));
    EXPECT(assert_equal("p1", arg1.name));
    EXPECT(arg1.type.namespaces_.empty());

    Argument arg2 = args.at(1);
    EXPECT(arg2.is_ptr);
    EXPECT(!arg2.is_const);
    EXPECT(!arg2.is_ref);
    EXPECT(assert_equal("Test", arg2.type.name_));
    EXPECT(assert_equal("p2", arg2.name));
    EXPECT(arg2.type.namespaces_.empty());
  }

  // evaluate global functions
  //  Vector aGlobalFunction();
  LONGS_EQUAL(2, module.global_functions.size());
  CHECK(module.global_functions.find("aGlobalFunction") != module.global_functions.end());
  {
    GlobalFunction gfunc = module.global_functions.at("aGlobalFunction");
    EXPECT(assert_equal("aGlobalFunction", gfunc.name()));
    LONGS_EQUAL(1, gfunc.nrOverloads());
    EXPECT(assert_equal("Vector", gfunc.returnValue(0).type1.name()));
    EXPECT_LONGS_EQUAL(1, gfunc.nrOverloads());
    LONGS_EQUAL(1, gfunc.overloads.size());
    EXPECT(gfunc.overloads.front().namespaces().empty());
  }
}

/* ************************************************************************* */
TEST( wrap, parse_namespaces ) {
  string header_path = topdir + "/wrap/tests";
  Module module(header_path.c_str(), "testNamespaces",enable_verbose);
  EXPECT_LONGS_EQUAL(6, module.classes.size());

  {
    strvec module_exp_includes;
    module_exp_includes += "path/to/ns1.h";
    module_exp_includes += "path/to/ns1/ClassB.h";
    module_exp_includes += "path/to/ns2.h";
    module_exp_includes += "path/to/ns2/ClassA.h";
    module_exp_includes += "path/to/ns3.h";
    EXPECT(assert_equal(module_exp_includes, module.includes));
  }

  {
    Class cls = module.classes.at(0);
    EXPECT(assert_equal("ClassA", cls.name()));
    strvec exp_namespaces; exp_namespaces += "ns1";
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  {
    Class cls = module.classes.at(1);
    EXPECT(assert_equal("ClassB", cls.name()));
    strvec exp_namespaces; exp_namespaces += "ns1";
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  {
    Class cls = module.classes.at(2);
    EXPECT(assert_equal("ClassA", cls.name()));
    strvec exp_namespaces; exp_namespaces += "ns2";
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  {
    Class cls = module.classes.at(3);
    EXPECT(assert_equal("ClassB", cls.name()));
    strvec exp_namespaces; exp_namespaces += "ns2", "ns3";
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  {
    Class cls = module.classes.at(4);
    EXPECT(assert_equal("ClassC", cls.name()));
    strvec exp_namespaces; exp_namespaces += "ns2";
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  {
    Class cls = module.classes.at(5);
    EXPECT(assert_equal("ClassD", cls.name()));
    strvec exp_namespaces;
    EXPECT(assert_equal(exp_namespaces, cls.namespaces()));
  }

  // evaluate global functions
//  Vector ns1::aGlobalFunction();
//  Vector ns2::aGlobalFunction();
  LONGS_EQUAL(2, module.global_functions.size());
  CHECK(module.global_functions.find("aGlobalFunction") != module.global_functions.end());
  {
    GlobalFunction gfunc = module.global_functions.at("aGlobalFunction");
    EXPECT(assert_equal("aGlobalFunction", gfunc.name()));
    LONGS_EQUAL(2, gfunc.nrOverloads());
    EXPECT(assert_equal("Vector", gfunc.returnValue(0).type1.name()));

    // check namespaces
    LONGS_EQUAL(2, gfunc.overloads.size());
    strvec exp_namespaces1; exp_namespaces1 += "ns1";
    EXPECT(assert_equal(exp_namespaces1, gfunc.overloads.at(0).namespaces()));

    strvec exp_namespaces2; exp_namespaces2 += "ns2";
    EXPECT(assert_equal(exp_namespaces2, gfunc.overloads.at(1).namespaces()));
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
  module.matlab_code("actual_namespaces");


  EXPECT(files_equal(exp_path + "ClassD.m", act_path + "ClassD.m" ));
  EXPECT(files_equal(exp_path + "+ns1/ClassA.m", act_path + "+ns1/ClassA.m" ));
  EXPECT(files_equal(exp_path + "+ns1/ClassB.m", act_path + "+ns1/ClassB.m" ));
  EXPECT(files_equal(exp_path + "+ns2/ClassA.m", act_path + "+ns2/ClassA.m" ));
  EXPECT(files_equal(exp_path + "+ns2/ClassC.m", act_path + "+ns2/ClassC.m" ));
  EXPECT(
      files_equal(exp_path + "+ns2/overloadedGlobalFunction.m", exp_path + "+ns2/overloadedGlobalFunction.m" ));
  EXPECT(
      files_equal(exp_path + "+ns2/+ns3/ClassB.m", act_path + "+ns2/+ns3/ClassB.m" ));
  EXPECT(
      files_equal(exp_path + "testNamespaces_wrapper.cpp", act_path + "testNamespaces_wrapper.cpp" ));
}

/* ************************************************************************* */
TEST( wrap, matlab_code_geometry ) {
  // Parse into class object
  string header_path = topdir + "/wrap/tests";
  Module module(header_path,"geometry",enable_verbose);
  string path = topdir + "/wrap";

  // clean out previous generated code
  fs::remove_all("actual");

  // emit MATLAB code
  // make_geometry will not compile, use make testwrap to generate real make
  module.matlab_code("actual");
#ifndef WRAP_DISABLE_SERIALIZE
  string epath = path + "/tests/expected/";
#else
  string epath = path + "/tests/expected2/";
#endif
  string apath = "actual/";

  EXPECT(files_equal(epath + "geometry_wrapper.cpp" , apath + "geometry_wrapper.cpp" ));
  EXPECT(files_equal(epath + "+gtsam/Point2.m"      , apath + "+gtsam/Point2.m"      ));
  EXPECT(files_equal(epath + "+gtsam/Point3.m"      , apath + "+gtsam/Point3.m"      ));
  EXPECT(files_equal(epath + "Test.m"               , apath + "Test.m"               ));
  EXPECT(files_equal(epath + "MyBase.m"             , apath + "MyBase.m"             ));
  EXPECT(files_equal(epath + "MyVector3.m"          , apath + "MyVector3.m"          ));
  EXPECT(files_equal(epath + "MyVector12.m"         , apath + "MyVector12.m"         ));
  EXPECT(files_equal(epath + "MyTemplatePoint2.m"   , apath + "MyTemplatePoint2.m"   ));
  EXPECT(files_equal(epath + "MyTemplateMatrix.m"   , apath + "MyTemplateMatrix.m"   ));
  EXPECT(files_equal(epath + "MyFactorPosePoint2.m" , apath + "MyFactorPosePoint2.m" ));
  EXPECT(files_equal(epath + "aGlobalFunction.m"    , apath + "aGlobalFunction.m"    ));
  EXPECT(files_equal(epath + "overloadedGlobalFunction.m", apath + "overloadedGlobalFunction.m"));
}

/* ************************************************************************* */
TEST( wrap, python_code_geometry ) {
  // Parse into class object
  string header_path = topdir + "/wrap/tests";
  Module module(header_path,"geometry",enable_verbose);
  string path = topdir + "/wrap";

  // clean out previous generated code
  fs::remove_all("actual-python");

  // emit MATLAB code
  // make_geometry will not compile, use make testwrap to generate real make
  module.python_wrapper("actual-python");
  string epath = path + "/tests/expected-python/";
  string apath = "actual-python/";

  EXPECT(files_equal(epath + "geometry_python.cpp", apath + "geometry_python.cpp" ));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
