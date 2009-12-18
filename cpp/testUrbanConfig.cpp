/*
 * @file testUrbanConfig.cpp
 * @brief Tests for the CitySLAM configuration class
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <UrbanConfig.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( UrbanConfig, update_with_large_delta)
{
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
