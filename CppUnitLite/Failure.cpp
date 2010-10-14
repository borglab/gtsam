/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */



#include "Failure.h"

#include <stdio.h>
#include <string.h> 


Failure::Failure (const SimpleString&	theTestName, 
				  const SimpleString&	theFileName, 
		          long	 				theLineNumber,
		          const SimpleString&	theCondition) 
: message (theCondition), 
  testName (theTestName), 
  fileName (theFileName), 
  lineNumber (theLineNumber)
{
}

Failure::Failure (const SimpleString&	theTestName,
				  const SimpleString&	theFileName,
		          const SimpleString&	theCondition)
: message (theCondition),
  testName (theTestName),
  fileName (theFileName),
  lineNumber (-1)
{
}


Failure::Failure (const SimpleString&	theTestName, 
			 	  const SimpleString&	theFileName, 
				  long					theLineNumber,
				  const SimpleString&	expected,
				  const SimpleString&	actual) 
: testName (theTestName), 
  fileName (theFileName), 
  lineNumber (theLineNumber)
{
	const char *part1 = "expected ";
	const char *part3 = " but was: ";

	char *stage = new char [strlen (part1) 
					+ expected.size () 
					+ strlen (part3)
					+ actual.size ()
					+ 1];

	sprintf(stage, "%s%s%s%s", 
		part1, 
		expected.asCharString(), 
		part3, 
		actual.asCharString());

	message = SimpleString(stage);

	delete stage;
}


