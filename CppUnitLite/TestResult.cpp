
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
		fprintf (stdout, "%s%s%s%s%ld%s%s\n",
				"Failure: \"",
				failure.message.asCharString (),
				"\" in ",
				failure.fileName.asCharString ());
	else
		fprintf (stdout, "%s%s%s%s%ld%s%s\n",
				"Failure: \"",
				failure.message.asCharString (),
				"\" " ,
				"line ",
				failure.lineNumber,
				" in ",
				failure.fileName.asCharString ());

	failureCount++;
}


void TestResult::testsEnded () 
{
	if (failureCount > 0)
		fprintf (stdout, "There were %d failures\n", failureCount);
	else
		fprintf (stdout, "There were no test failures\n");
}
