/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


#include "Test.h"
#include "TestRegistry.h"
#include "TestResult.h"
#include "Failure.h"

#include <boost/lexical_cast.hpp>

Test::Test (const std::string& testName)
  : name_ (testName), next_(0), lineNumber_(-1), safeCheck_(true)
{
  TestRegistry::addTest (this);
}

Test::Test (const std::string& testName, const std::string& filename, long lineNumber, bool safeCheck)
  : name_(testName), next_(0), filename_(filename), lineNumber_(lineNumber), safeCheck_(safeCheck)
{
  TestRegistry::addTest (this);
}


Test *Test::getNext() const
{
  return next_;
}

void Test::setNext(Test *test)
{
  next_ = test;
}

bool Test::check(long expected, long actual, TestResult& result, const std::string& fileName, long lineNumber)
{
  if (expected == actual)
    return true;
  result.addFailure (
    Failure (
      name_,
      boost::lexical_cast<std::string> (__FILE__),
      __LINE__,
      boost::lexical_cast<std::string> (expected),
      boost::lexical_cast<std::string> (actual)));

  return false;

}


bool Test::check(const std::string& expected, const std::string& actual, TestResult& result, const std::string& fileName, long lineNumber)
{
  if (expected == actual)
    return true;
  result.addFailure (
    Failure (
      name_,
      boost::lexical_cast<std::string> (__FILE__),
      __LINE__,
      expected,
      actual));

  return false;

}

