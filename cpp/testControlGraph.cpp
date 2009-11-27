/**
 * @file testControlGraph.cpp
 * @author Alex Cunningham
 */

#include <cmath>
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/map.hpp> // for insert
#include <CppUnitLite/TestHarness.h>
#include "ControlGraph.h"
#include "ControlPoint.h"
#include "Ordering.h"
#include "SQPOptimizer.h"
#include "NonlinearEquality.h"

// implementations
#include "SQPOptimizer-inl.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

typedef SQPOptimizer<ControlGraph, ControlConfig> COptimizer;

/* ************************************************************************* */
TEST (ControlGraph, add_agents) {
	// create a graph with a robot with performance parameters
	ControlGraph graph;
	double maxVel=2.0, maxAcc=1.0, maxRotVel=3.0, maxRotAcc=1.0;
	graph.addAgent("r1", maxVel, maxAcc, maxRotVel, maxRotAcc);

	// read the robot information back out
	CHECK(graph.nrAgents() == 1);
	ControlGraph::DynamicsModel
		actModel = graph.agentModel("r1"),
		expModel(maxVel, maxAcc, maxRotVel, maxRotAcc);
	CHECK(assert_equal(actModel, expModel));

	// initialize a robot directly with a model
	ControlGraph::DynamicsModel model2(1.0, 2.0, 3.0, 4.0), actModel2;
	graph.addAgent("r2", model2);
	CHECK(graph.nrAgents() == 2);
	actModel2 = graph.agentModel("r2");
	CHECK(assert_equal(actModel2, model2));

	// get the names of the agents
	set<string> actAgents = graph.agents(), expAgents;
	expAgents += "r1", "r2";
	CHECK(expAgents.size() == actAgents.size());
}

/* ************************************************************************* */
TEST (ControlGraph, equals) {
	ControlGraph fg1, fg2;
	CHECK(assert_equal(fg1, fg2));
	fg1.addAgent("r1", 1.0, 1.0, 1.0, 1.0);
	fg1.addAgent("r2", 1.0, 1.0, 1.0, 1.0);
	CHECK(!fg1.equals(fg2));
	fg2.addAgent("r1", 1.0, 1.0, 1.0, 1.0);
	fg2.addAgent("r2", 1.0, 1.0, 1.0, 1.0);
	CHECK(assert_equal(fg1, fg2));
}

/* ************************************************************************* */
TEST (ControlGraph, fix_constraints) {

	// create a graph with a single robot
	ControlGraph graph;
	double maxVel, maxAcc;
	maxVel = maxAcc = sqrt(2.0)+0.2;
	graph.addAgent("r1", maxVel, maxAcc, 10.0, 10.0);

	// constrain the ends
	ControlPoint start,
				  end(Pose2(2.0, 2.0, 0.0), Pose2(), 2.0);
	graph.fixAgentState("r1", start, 0);
	graph.fixAgentState("r1", end, 2);

	// extract the constraints
	typedef NonlinearEquality<ControlConfig> NLE;
	typedef NonlinearFactor<ControlConfig> NLF;
	boost::shared_ptr<NLF> cStart = graph[0], cEnd = graph[1];
	boost::shared_ptr<NLE> actStart = boost::shared_dynamic_cast<NLE>(cStart);
	boost::shared_ptr<NLE> actEnd = boost::shared_dynamic_cast<NLE>(cEnd);

	// fetch feasible set from graph
	ControlConfig feasible = graph.feasible();

	// create expected values
	NLE expStart("r1_0", feasible, 7, ControlConfig::compareConfigState);
	NLE expEnd("r1_2", feasible, 7, ControlConfig::compareConfigState);
	CHECK(assert_equal(expStart, *actStart));
	CHECK(assert_equal(expEnd, *actEnd));
}

/* ************************************************************************* */
TEST (ControlGraph, automated_graphgen_optimization) {
	// create a graph with a robot with performance parameters
	ControlGraph graph;
	double maxVel=2.0, maxAcc=1.0, maxRotVel=3.0, maxRotAcc=1.0;
	graph.addAgent("r1", maxVel, maxAcc, maxRotVel, maxRotAcc);

	// fix initial states for the agents
	graph.fixAgentState("r1", ControlPoint()); // default argument fixes start

	// create a three-state trajectory (4 including the initial state)
	graph.addTrajectory("r1", 3);

	// fix the end of the trajectory
	ControlPoint endPt(Pose2(5.0, 0.0, 0.0), Pose2(), 5.0);
	graph.fixAgentState("r1", endPt, 3);

	// create an initial config
	ControlConfig::shared_config initConfig(new ControlConfig);
	initConfig->addAgent("r1");
	initConfig->addPoint("r1", ControlPoint());
	initConfig->addPoint("r1", ControlPoint());
	initConfig->addPoint("r1", ControlPoint());
	initConfig->addPoint("r1", endPt);

	// create an ordering
	Ordering ordering;
	ordering += "r1_0", "r1_1", "r1_2", "r1_3";

//	// create an optimizer
//	COptimizer optimizer(graph, ordering, initConfig);
//
//	// do an iteration
//	COptimizer oneIteration = optimizer.iterate(COptimizer::FULL);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
