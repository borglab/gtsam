/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

///////////////////////////////////////////////////////////////////////////////
//
// TEST.H
// 
// This file contains the Test class along with the macros which make effective
// in the harness.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TEST_H
#define TEST_H


#include <cmath>
#include "SimpleString.h"

class TestResult;



class Test
{
public:
	Test (const SimpleString& testName);
	Test (const SimpleString& testName, const SimpleString& filename, long lineNumber, bool safeCheck);
  virtual ~Test() {};

	virtual void	run (TestResult& result) = 0;


	void			setNext(Test *test);
	Test			*getNext () const;
	SimpleString    getName() const {return name_;}
	SimpleString 	getFilename() const {return filename_;}
	long			getLineNumber() const {return lineNumber_;}
	bool  			safe() const {return safeCheck_;}

protected:

	bool check (long expected, long actual, TestResult& result, const SimpleString& fileName, long lineNumber);
	bool check (const SimpleString& expected, const SimpleString& actual, TestResult& result, const SimpleString& fileName, long lineNumber);

	SimpleString	name_;
	Test			*next_;
	SimpleString 	filename_;
	long 			lineNumber_; /// This is the line line number of the test, rather than the a single check
	bool 			safeCheck_;

};

/**
 * Normal test will wrap execution in a try/catch block to catch exceptions more effectively
 */
#define TEST(testName, testGroup)\
  class testGroup##testName##Test : public Test \
	{ public: testGroup##testName##Test () : Test (#testName "Test", __FILE__, __LINE__, true) {} \
            void run (TestResult& result_);} \
    testGroup##testName##Instance; \
	void testGroup##testName##Test::run (TestResult& result_) 

/**
 * For debugging only: use TEST_UNSAFE to allow debuggers to have access to exceptions, as this
 * will not wrap execution with a try/catch block
 */
#define TEST_UNSAFE(testName, testGroup)\
  class testGroup##testName##Test : public Test \
	{ public: testGroup##testName##Test () : Test (#testName "Test", __FILE__, __LINE__, false) {} \
            void run (TestResult& result_);} \
    testGroup##testName##Instance; \
	void testGroup##testName##Test::run (TestResult& result_)

/*
 * Convention for tests:
 *  - "EXPECT" is a test that will not end execution of the series of tests
 *  - Otherwise, upon a failure, the test will end
 *
 * Usage:
 *  EXPECT is useful when checking several different parts of an condition so
 *  that a failure of one check won't hide another failure.
 *
 * Note: Exception tests are not available in a EXPECT form, as exceptions rarely
 * fit the criteria of an assertion that does not need to be true to continue
 */

/* True ASSERTs: tests end at first failure */
#define CHECK(condition)\
{ if (!(condition)) \
{ result_.addFailure (Failure (name_, __FILE__,__LINE__, #condition)); return; } }

#define THROWS_EXCEPTION(condition)\
{ try { condition; \
		result_.addFailure (Failure (name_, __FILE__,__LINE__, SimpleString("Didn't throw: ") + StringFrom(#condition))); \
		return; } \
  catch (...) {} }

#define CHECK_EXCEPTION(condition, exception_name)\
{ try { condition; \
		result_.addFailure (Failure (name_, __FILE__,__LINE__, SimpleString("Didn't throw: ") + StringFrom(#condition))); \
		return; } \
  catch (exception_name& e) {} \
  catch (...) { \
	result_.addFailure (Failure (name_, __FILE__,__LINE__, SimpleString("Wrong exception: ") + StringFrom(#condition) + StringFrom(", expected: ") + StringFrom(#exception_name))); \
	return; } }

#define EQUALITY(expected,actual)\
  { if (!assert_equal(expected,actual)) \
    result_.addFailure(Failure(name_, __FILE__, __LINE__, #expected, #actual)); }

#define CHECK_EQUAL(expected,actual)\
{ if ((expected) == (actual)) return; result_.addFailure(Failure(name_, __FILE__, __LINE__, StringFrom(expected), StringFrom(actual))); }

#define LONGS_EQUAL(expected,actual)\
{ long actualTemp = actual; \
  long expectedTemp = expected; \
  if ((expectedTemp) != (actualTemp)) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, StringFrom(expectedTemp), \
StringFrom(actualTemp))); return; } }

#define DOUBLES_EQUAL(expected,actual,threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (fabs ((expectedTemp)-(actualTemp)) > threshold) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, \
StringFrom((double)expectedTemp), StringFrom((double)actualTemp))); return; } }


/* EXPECTs: tests will continue running after a failure */
#define EXPECT(condition)\
{ if (!(condition)) \
{ result_.addFailure (Failure (name_, __FILE__,__LINE__, #condition)); } }

#define EXPECT_LONGS_EQUAL(expected,actual)\
{ long actualTemp = actual; \
  long expectedTemp = expected; \
  if ((expectedTemp) != (actualTemp)) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, StringFrom(expectedTemp), \
StringFrom(actualTemp))); } }

#define EXPECT_DOUBLES_EQUAL(expected,actual,threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (fabs ((expectedTemp)-(actualTemp)) > threshold) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, \
StringFrom((double)expectedTemp), StringFrom((double)actualTemp))); } }


#define FAIL(text) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__,(text))); return; }



#endif
