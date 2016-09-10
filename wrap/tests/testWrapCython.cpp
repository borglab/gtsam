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
static bool enable_verbose = true;
#ifdef TOPSRCDIR
static string topdir = TOPSRCDIR;
#else
static string topdir = "TOPSRCDIR_NOT_CONFIGURED"; // If TOPSRCDIR is not defined, we error
#endif

/* ************************************************************************* */
TEST( wrap, cython_code_geometry ) {
  // Parse into class object
  string header_path = topdir + "/wrap/tests";
  Module module(header_path,"cythontest",enable_verbose);
  string path = topdir + "/wrap";

  // clean out previous generated code
  fs::remove_all("actual-cython");

  // emit MATLAB code
  // make_geometry will not compile, use make testwrap to generate real make
  module.cython_wrapper("actual-cython");
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
