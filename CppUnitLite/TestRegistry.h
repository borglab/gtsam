/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

///////////////////////////////////////////////////////////////////////////////
//
// TESTREGISTRY.H
//
// TestRegistry is a singleton collection of all the tests to run in a system.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TESTREGISTRY_H
#define TESTREGISTRY_H


class Test;
class TestResult;



class TestRegistry
{
public:
  static void addTest (Test *test);
  static int runAllTests (TestResult& result);

private:

  static TestRegistry&  instance ();
  void          add (Test *test);
  int           run (TestResult& result);


  Test          *tests;
  Test          *lastTest;

};




#endif
