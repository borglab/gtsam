/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "TestResult.h"
#include "Failure.h"

#include <stdio.h>


TestResult::TestResult ()
  : failureCount (0)
{
}


void TestResult::testsStarted ()
{
}


void TestResult::addFailure (const Failure& failure)
{
  if (failure.lineNumber < 0) // allow for no line number
    fprintf (stdout, "%s%s%s%s\n",
        "Failure: \"",
        failure.message.c_str (),
        "\" in ",
        failure.fileName.c_str ());
  else
    fprintf (stdout, "%s%s%ld%s%s%s\n",
        failure.fileName.c_str(),  // Format matches Eclipse error flagging
        ":",
        failure.lineNumber,
        ": Failure: \"",
        failure.message.c_str(),
        "\" ");

  failureCount++;
}


void TestResult::testsEnded ()
{
  if (failureCount > 0)
    fprintf (stdout, "There were %d failures\n", failureCount);
  else
    fprintf (stdout, "There were no test failures\n");
}
