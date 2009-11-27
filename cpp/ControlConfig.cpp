/**
 * @file ControlConfig.cpp
 * @brief Implementation of ControlConfig
 * @author Alex Cunningham
 */

#include <iostream>
#include <sstream>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include "ControlConfig.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

// convert to strings
template<typename T>
string toStr(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}

/* *************************************************************** */
void ControlConfig::print(const std::string& name) const {
	cout << "Config: " << name << endl;
	string agent; path p;
	FOREACH_PAIR(agent, p, paths_) {
		cout << "Agent: " << agent << "\n";
		int i = 0;
		BOOST_FOREACH(ControlPoint pt, p) {
			ostringstream oss;
			oss << "Point: " << i++;
			pt.print(oss.str());
		}
		cout << endl;
	}
}

/* *************************************************************** */
bool ControlConfig::equals(const ControlConfig& expected, double tol) const {
	if (paths_.size() != expected.paths_.size()) return false;
	string j; path pa;
	FOREACH_PAIR(j, pa, paths_) {
		if (!expected.involvesAgent(j))
			return false;
		path pb = expected.getPath(j);
		if (pa.size() != pb.size())
			return false;
		for (int i=0; i<pa.size(); ++i)
			if (!pa.at(i).equals(pb.at(i), tol))
				return false;
	}
	return true;
}

/* *************************************************************** */
void ControlConfig::addAgent(const std::string& name) {
	if (paths_.find(name) == paths_.end()) {
		path p;
		paths_.insert(make_pair(name, p));
	}
}

/* *************************************************************** */
void ControlConfig::addPoint(const std::string& name, const ControlPoint& state, int index) {
	if (index < -1 )
		throw invalid_argument("Attempting to add point before start of trajectory");
	if (paths_.find(name) != paths_.end()) {
		path &p = paths_[name];
		if (index == -1) {
			// just add the point to the back of the trajectory
			p.push_back(state);
		} else if (index < p.size()) {
			// insert to existing point
			p[index] = state;
		} else {
			// pad the trajectory to a particular size
			p.resize(index+1);
			p[index] = state;
		}
	} else {
		throw invalid_argument("Attempting to add point without existing agent");
	}
}

/* *************************************************************** */
ControlConfig::path ControlConfig::getPath(const std::string& agentID) const {
	const_iterator it = paths_.find(agentID);
	if (it != paths_.end()) {
		return it->second;
	} else {
		throw invalid_argument("Attempting to access path that does not exist");
	}
}

/* *************************************************************** */
bool ControlConfig::involvesAgent(const std::string& agentID) const {
	return paths_.find(agentID) != paths_.end();
}

/* *************************************************************** */
void ControlConfig::clearAgent(const std::string& agentID) {
	const_iterator it = paths_.find(agentID);
	if (it != paths_.end()) {
		path &p = paths_[agentID];
		p.clear();
	} else {
		throw invalid_argument("Attempting to clear agent that is not present");
	}
}

/* *************************************************************** */
ControlConfig ControlConfig::exmap(const VectorConfig & delta) const {
	ControlConfig newConfig; string agent; path p;
	FOREACH_PAIR(agent, p, paths_) {
		newConfig.addAgent(agent);
		for (size_t i=0; i<p.size(); ++i) {
			string key = agent + "_" + toStr(i);
			ControlPoint newPt = p.at(i).exmap(delta[key]);
			newConfig.addPoint(agent, newPt);
		}
	}
	return newConfig;
}

/* *************************************************************** */
string ControlConfig::nameGen(const string& name, size_t num) {
	return name + "_" + toStr(num);
}

/* *************************************************************** */
bool ControlConfig::compareConfigState(const std::string& key,
			const ControlConfig& feasible, const ControlConfig& input) {
	return feasible.get(key).equals(input.get(key));
}

/* *************************************************************** */
ControlPoint ControlConfig::get(const std::string& key) const {
	size_t delim = key.find_first_of('_');
	string agent = key.substr(0, delim);
	int num = atoi(key.substr(delim+1).c_str());
	return getPath(agent).at(num);
}
