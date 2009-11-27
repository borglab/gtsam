/**
 * @file ControlGraph.cpp
 * @brief Implementation of a graph for solving robot control problems
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/assign/list_inserter.hpp>
#include <boost/tuple/tuple.hpp>
#include "ControlGraph.h"
#include "NonlinearEquality.h"
#include "NonlinearFactorGraph-inl.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

/* ************************************************************************* */
void ControlGraph::print(const std::string& name) const {
	gtsam::NonlinearFactorGraph<ControlConfig>::print(name);
}

/* ************************************************************************* */
bool ControlGraph::equals(const ControlGraph& p, double tol) const {
	if (&p == NULL) return false;
	if (models_.size() != p.models_.size()) return false;
	const_model_it it1 = models_.begin(), it2 = p.models_.begin();
	for (; it1 != models_.end(), it2 != p.models_.end(); ++it1, ++it2) {
		if (it1->first != it2->first) return false;
		if (!it1->second.equals(it2->second)) return false;
	}
	return true;
}

/* ************************************************************************* */
void ControlGraph::addAgent(const std::string& name,
							  double maxVel, double maxAcc,
							  double maxRotVel, double maxRotAcc) {
	addAgent(name,
			 ControlGraph::DynamicsModel(maxVel, maxAcc, maxRotVel, maxRotAcc));
}

/* ************************************************************************* */
void ControlGraph::addAgent(const std::string& name, const ControlGraph::DynamicsModel& model) {
	insert(models_)(name, model);
}

/* ************************************************************************* */
ControlGraph::DynamicsModel ControlGraph::agentModel(const std::string& agent) const {
	const_model_it it = models_.find(agent);
	if (it != models_.end())
		return it->second;
	else
		throw invalid_argument("Attempting to access invalid agent: " + agent);
}

/* ************************************************************************* */
void ControlGraph::addTrajectory(const std::string& name, size_t states) {
	//TODO: Implement this function

	// for each node to add

	// add a temporal bounding constraint (first node is before second)
	// add a path shortening factor (move points closer)
	// add maximum velocity factor
	// add velocity and acceleration clamping
}

/* ************************************************************************* */
void ControlGraph::fixAgentState(const std::string& name,
		const ControlPoint& state, size_t state_num) {
	// add a nonlinear equality constraint
	typedef NonlinearEquality<ControlConfig> NLE;
	feasible_.addAgent(name);
	feasible_.addPoint(name, state, state_num);
	boost::shared_ptr<NLE> constraint(new NLE(ControlConfig::nameGen(name, state_num),
			feasible_, 7, ControlConfig::compareConfigState));
	push_back(constraint);
}

/* ************************************************************************* */
set<string> ControlGraph::agents() const {
	set<string> ret;
	string key; ControlGraph::DynamicsModel m;
	FOREACH_PAIR(key, m, models_) {
		insert(ret)(key);
	}
	return ret;
}


/* ************************************************************************* */
// Implementation of DynamicsModel
/* ************************************************************************* */
ControlGraph::DynamicsModel::DynamicsModel()
: maxVel_(100.0), maxAcc_(100.0), maxRotVel_(100.0), maxRotAcc_(100.0)
{
}

/* ************************************************************************* */
ControlGraph::DynamicsModel::DynamicsModel(double maxVel, double maxAcc,
		  double maxRotVel, double maxRotAcc)
: maxVel_(maxVel), maxAcc_(maxAcc), maxRotVel_(maxRotVel), maxRotAcc_(maxRotAcc)
{
}

/* ************************************************************************* */
void ControlGraph::DynamicsModel::print(const std::string& name) const {
	cout << "Dynamics Model: " << name << "\n"
	     << "   maxVel: " << maxVel_ << "\n"
	     << "   maxAcc: " << maxAcc_ << "\n"
	     << "   maxRotVel: " << maxRotVel_ << "\n"
	     << "   maxRotAcc: " << maxRotAcc_ << endl;
}

/* ************************************************************************* */
bool ControlGraph::DynamicsModel::equals(const DynamicsModel& m, double tol) const {
	return maxVel_ == m.maxVel_ &&
		   maxAcc_ == m.maxAcc_ &&
		   maxRotVel_ == m.maxRotVel_ &&
		   maxRotAcc_ == m.maxRotAcc_;
}
