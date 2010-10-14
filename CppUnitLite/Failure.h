/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


///////////////////////////////////////////////////////////////////////////////
//
// FAILURE.H
//
// Failure is a class which holds information pertaining to a specific
// test failure.  The stream insertion operator is overloaded to allow easy
// display.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef FAILURE_H
#define FAILURE_H

#include "SimpleString.h"


class Failure
{

public:
	Failure (const SimpleString&		theTestName, 
			 const SimpleString&		theFileName, 
			 long	  					theLineNumber,
			 const SimpleString&		theCondition);

	Failure (const SimpleString&		theTestName, 
			 const SimpleString&		theFileName, 
			 long						theLineNumber,
			 const SimpleString&		expected,
			 const SimpleString&		actual);

	Failure (const SimpleString&		theTestName,
			 const SimpleString&		theFileName,
			 const SimpleString&		theCondition);


	SimpleString		message;
	SimpleString		testName;
	SimpleString		fileName;
	long				lineNumber;
};


#endif
