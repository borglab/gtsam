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

#include <string>

class Failure
{

public:
	Failure (const std::string&		theTestName,
			 const std::string&		theFileName,
			 long	  					theLineNumber,
			 const std::string&		theCondition);

	Failure (const std::string&		theTestName,
			 const std::string&		theFileName,
			 long						theLineNumber,
			 const std::string&		expected,
			 const std::string&		actual);

	Failure (const std::string&		theTestName,
			 const std::string&		theFileName,
			 const std::string&		theCondition);


	std::string		message;
	std::string		testName;
	std::string		fileName;
	long				lineNumber;
};


#endif
