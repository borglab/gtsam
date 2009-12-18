
/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "Pose2Graph.h"
#include "NonlinearFactorGraph-inl.h"
#include "GaussianFactorGraph.h"

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
	Pose2Factor constraint("x1","x2",measured, measurement_covariance);
	Pose2Graph graph;
	graph.push_back(Pose2Factor::shared_ptr(new Pose2Factor("x1","x2",measured, measurement_covariance)));
	// get the size of the graph
	size_t actual = graph.size();
	// verify
	size_t expected = 1;
	CHECK(actual == expected);

}
TEST( Pose2Graph, linerization )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Matrix measurement_covariance = Matrix_(3,3,
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.01
	);
	Pose2Factor constraint("x1","x2",measured, measurement_covariance);
	Pose2Graph graph;
	graph.push_back(Pose2Factor::shared_ptr(new Pose2Factor("x1","x2",measured, measurement_covariance)));

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4) looking at negative (ground truth is at 4.1,2)
	Pose2Config config;
	config.insert("x1",p1);
	config.insert("x2",p2);
	// Linearize
	GaussianFactorGraph lfg_linearized = graph.linearize(config);
	//lfg_linearized.print("lfg_actual");

	// the expected linear factor
	GaussianFactorGraph lfg_expected;
	Matrix A1 = Matrix_(3,3,
			0.0,	2.0, -4.2,
			-2.0,	0.0,  4.2,
			0.0,	0.0,  10.0);

	Matrix A2 = Matrix_(3,3,
			0.0,	-2.0,  0.0,
			2.0,	0.0,   0.0,
			0.0,	0.0,  -10.0);


	double sigma = 1;
	Vector b = Vector_(3,0.1,0.1,0.0);
	lfg_expected.add("x1", A1, "x2", A2, b, sigma);


	CHECK(lfg_expected.equals(lfg_linearized));

}
/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
