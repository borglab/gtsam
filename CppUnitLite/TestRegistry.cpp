

#include <exception>

#include "Test.h"
#include "Failure.h"
#include "TestResult.h"
#include "TestRegistry.h"
#include "SimpleString.h"


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
		tests = test;
		return;
	}

	test->setNext (tests);
	tests = test;
}


int TestRegistry::run (TestResult& result) 
{
	result.testsStarted ();

	for (Test *test = tests; test != 0; test = test->getNext ()) {
		try {
			test->run (result);
		} catch (std::exception& e) {
			// catch standard exceptions and derivatives
			result.addFailure(
					Failure(test->getName(), test->getFilename(), test->getLineNumber(),
							SimpleString("Exception: ") + SimpleString(e.what())));
		} catch (...) {
			// catch all other exceptions
			result.addFailure(
					Failure(test->getName(), test->getFilename(), test->getLineNumber(),
							SimpleString("ExceptionThrown!")));
		}
	}
	result.testsEnded ();
	return result.getFailureCount();
}



