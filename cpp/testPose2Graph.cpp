
/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;


TEST( Pose2Graph, constructor )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Matrix measurement_covariance = Matrix_(3,3,
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.01
			);
	Pose2Factor constraint("p1","p2",measured, measurement_covariance);
	Pose2Graph graph;
	graph.push_back(Pose2Factor::shared_ptr(new Pose2Factor("p1","p2",measured, measurement_covariance)));
	// get the size of the graph
	size_t actual = graph.size();
	// verify
	size_t expected = 1;
	CHECK(actual == expected);

}
/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
