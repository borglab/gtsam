/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */



#include <exception>

#include "Test.h"
#include "Failure.h"
#include "TestResult.h"
#include "TestRegistry.h"

void TestRegistry::addTest (Test *test)
{
  instance ().add (test);
}


int TestRegistry::runAllTests (TestResult& result)
{
  return instance ().run (result);
}


TestRegistry& TestRegistry::instance ()
{
  static TestRegistry registry;
  return registry;
}


void TestRegistry::add (Test *test)
{
  if (tests == 0) {
    test->setNext(0);
    tests = test;
    lastTest = test;
    return;
  }

  test->setNext (0);
  lastTest->setNext(test);
  lastTest = test;
}


int TestRegistry::run (TestResult& result)
{
  result.testsStarted ();

  for (Test *test = tests; test != 0; test = test->getNext ()) {
    if (test->safe()) {
      try {
        test->run (result);
      } catch (std::exception& e) {
        // catch standard exceptions and derivatives
        result.addFailure(
            Failure(test->getName(), test->getFilename(), test->getLineNumber(),
                std::string("Exception: ") + std::string(e.what())));
      } catch (...) {
        // catch all other exceptions
        result.addFailure(
            Failure(test->getName(), test->getFilename(), test->getLineNumber(),
                "ExceptionThrown!"));
      }
    }
    else {
      test->run (result);
    }
  }
  result.testsEnded ();
  return result.getFailureCount();
}



