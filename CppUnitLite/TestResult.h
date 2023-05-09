/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

///////////////////////////////////////////////////////////////////////////////
//
// TESTRESULT.H
//
// A TestResult is a collection of the history of some test runs.  Right now
// it just collects failures.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TESTRESULT_H
#define TESTRESULT_H

class Failure;

class TestResult
{
public:
          TestResult ();
  virtual ~TestResult() {};
  virtual void  testsStarted ();
  virtual void  addFailure (const Failure& failure);
  virtual void  testsEnded ();

  int getFailureCount() {return failureCount;}

private:
  int        failureCount;
};

#endif
