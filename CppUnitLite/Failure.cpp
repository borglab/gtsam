/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */



#include "Failure.h"

Failure::Failure (const std::string&	theTestName,
				  const std::string&	theFileName,
		          long	 				theLineNumber,
		          const std::string&	theCondition)
: message (theCondition), 
  testName (theTestName), 
  fileName (theFileName), 
  lineNumber (theLineNumber)
{
}

Failure::Failure (const std::string&	theTestName,
				  const std::string&	theFileName,
		          const std::string&	theCondition)
: message (theCondition),
  testName (theTestName),
  fileName (theFileName),
  lineNumber (-1)
{
}


Failure::Failure (const std::string&	theTestName,
			 	  const std::string&	theFileName,
				  long					theLineNumber,
				  const std::string&	expected,
				  const std::string&	actual)
: message("expected " + expected + " but was: " + actual),
  testName (theTestName),
  fileName (theFileName), 
  lineNumber (theLineNumber)
{
}


