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
#include <string>

class TestResult;

class Test
{
public:
  Test (const std::string& testName);
  Test (const std::string& testName, const std::string& filename, long lineNumber, bool safeCheck);
  virtual ~Test() {}

  virtual void  run (TestResult& result) = 0;


  void      setNext(Test *test);
  Test      *getNext () const;
  std::string    getName() const {return name_;}
  std::string   getFilename() const {return filename_;}
  long      getLineNumber() const {return lineNumber_;}
  bool        safe() const {return safeCheck_;}

protected:

  bool check (long expected, long actual, TestResult& result, const std::string& fileName, long lineNumber);
  bool check (const std::string& expected, const std::string& actual, TestResult& result, const std::string& fileName, long lineNumber);

  std::string  name_;
  Test      *next_;
  std::string   filename_;
  long       lineNumber_; /// This is the line line number of the test, rather than the a single check
  bool       safeCheck_;

};

/**
 * Normal test will wrap execution in a try/catch block to catch exceptions more effectively
 */
#define TEST(testGroup, testName)\
  class testGroup##testName##Test : public Test \
  { public: testGroup##testName##Test () : Test (#testName "Test", __FILE__, __LINE__, true) {} \
            void run (TestResult& result_) override;} \
    testGroup##testName##Instance; \
  void testGroup##testName##Test::run (TestResult& result_)

/**
 * Declare friend in a class to test its private methods
 */
#define FRIEND_TEST(testGroup, testName) \
    friend class testGroup##testName##Test;

/**
 * For debugging only: use TEST_UNSAFE to allow debuggers to have access to exceptions, as this
 * will not wrap execution with a try/catch block
 */
#define TEST_UNSAFE(testGroup, testName)\
  class testGroup##testName##Test : public Test \
  { public: testGroup##testName##Test () : Test (#testName "Test", __FILE__, __LINE__, false) {} \
            virtual ~testGroup##testName##Test () {};\
            void run (TestResult& result_) override;} \
    testGroup##testName##Instance; \
  void testGroup##testName##Test::run (TestResult& result_)

/**
 * Use this to disable unwanted tests without commenting them out.
 */
#define TEST_DISABLED(testGroup, testName)\
    void testGroup##testName##Test(TestResult& result_, const std::string& name_)

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
    result_.addFailure (Failure (name_, __FILE__,__LINE__, std::string("Didn't throw: ") + std::string(#condition))); \
    return; } \
  catch (...) {} }

#define CHECK_EXCEPTION(condition, exception_name)\
{ try { condition; \
    result_.addFailure (Failure (name_, __FILE__,__LINE__, std::string("Didn't throw: ") + std::string(#condition))); \
    return; } \
  catch (exception_name&) {} \
  catch (...) { \
  result_.addFailure (Failure (name_, __FILE__,__LINE__, std::string("Wrong exception: ") + std::string(#condition) + std::string(", expected: ") + std::string(#exception_name))); \
  return; } }

#define EQUALITY(expected,actual)\
  { if (!assert_equal(expected,actual)) \
    result_.addFailure(Failure(name_, __FILE__, __LINE__, #expected, #actual)); }

#define CHECK_EQUAL(expected,actual)\
{ if ((expected) == (actual)) return; result_.addFailure(Failure(name_, __FILE__, __LINE__, std::to_string(expected), std::to_string(actual))); }

#define LONGS_EQUAL(expected,actual)\
{ long actualTemp = actual; \
  long expectedTemp = expected; \
  if ((expectedTemp) != (actualTemp)) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, std::to_string(expectedTemp), \
std::to_string(actualTemp))); return; } }

#define DOUBLES_EQUAL(expected,actual,threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (!std::isfinite(actualTemp) || !std::isfinite(expectedTemp) || fabs ((expectedTemp)-(actualTemp)) > threshold) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, \
std::to_string((double)expectedTemp), std::to_string((double)actualTemp))); return; } }


/* EXPECTs: tests will continue running after a failure */
#define EXPECT(condition)\
{ if (!(condition)) \
{ result_.addFailure (Failure (name_, __FILE__,__LINE__, #condition)); } }

#define EXPECT_LONGS_EQUAL(expected,actual)\
{ long actualTemp = actual; \
  long expectedTemp = expected; \
  if ((expectedTemp) != (actualTemp)) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, std::to_string(expectedTemp), \
std::to_string(actualTemp))); } }

#define EXPECT_DOUBLES_EQUAL(expected,actual,threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (!std::isfinite(actualTemp) || !std::isfinite(expectedTemp) || fabs ((expectedTemp)-(actualTemp)) > threshold) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__, \
std::to_string((double)expectedTemp), std::to_string((double)actualTemp))); } }


#define FAIL(text) \
{ result_.addFailure (Failure (name_, __FILE__, __LINE__,(text))); return; }



#endif
