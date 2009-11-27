/**
 * @file ControlConfig.h
 * @brief Class to describe a configuration of 2D agents that use PV models
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <vector>
#include "Testable.h"
#include "ControlPoint.h"
#include "VectorConfig.h"

namespace gtsam {

/**
 * Class for configs of 2D agent motion models that make up trajectories
 * Provides a map of labeled robot poses, and means to access groups, such
 * as the trajectory of a particular robot or obstacle.
 */
class ControlConfig : public Testable<ControlConfig> {
public:
	/** allow for shared pointers */
	typedef boost::shared_ptr<ControlConfig> shared_config;

	/** an individual path object for an agent */
	typedef std::vector<ControlPoint> path;
	typedef std::map<std::string, path>::const_iterator const_iterator;

private:
	/** main storage for points */
	std::map<std::string, path> paths_;

public:
	/** Basic Default Constructors */
	ControlConfig() {}
	ControlConfig(const ControlConfig& cfg) : paths_(cfg.paths_) {}

	/** Default destructor */
	virtual ~ControlConfig() {}

	/** Standard print function with optional label */
	virtual void print(const std::string& name="") const;

	/** Equality up to a tolerance */
	virtual bool equals(const ControlConfig& expected, double tol=1e-9) const;

	/** Add a delta configuration to the config */
	ControlConfig exmap(const VectorConfig & delta) const;

	/** number of agents */
	size_t size() const { return paths_.size(); }

	/**
	 * Adds an agent to the config
	 * @param name is the name of the agent used for lookup
	 */
	void addAgent(const std::string& name);

	/**
	 * Adds a point to a robot's trajectory,
	 * note that ordering is handled internally
	 * @param name is the name of the robot
	 * @param state is the ControlPoint to add
	 * @param index is the index in the trajectory to insert the point (defaults to
	 * pushing to the back of the trajectory)
	 */
	void addPoint(const std::string& name, const ControlPoint& state, int index=-1);

	/**
	 * returns the path of a particular robot
	 */
	path getPath(const std::string& agentID) const;

	/** get a vector in the configuration by key */
	ControlPoint get(const std::string& key) const;

	/**
	 * Returns true if agent is in the config
	 */
	bool involvesAgent(const std::string& agentID) const;

	// clearing
	void clear() { paths_.clear(); }
	void clearAgent(const std::string& agentID);

	/**
	 * Generates a key for a key
	 * @param name is the name of the agent
	 * @param num is the sequence number of the robot
	 * @return a key in the form [name]_[num]
	 */
	static std::string nameGen(const std::string& name, size_t num);

	/**
	 * Compares two values of a config
	 * Used for creating NonlinearEqualities
	 * @param key identifier for the constrained variable
	 * @param feasible defines the feasible set
	 * @param input is the config to compare
	 * @return true if the selected value in feasible equals the input config
	 */
	static bool compareConfigState(const std::string& key,
			const ControlConfig& feasible, const ControlConfig& input);
};
} // \namespace gtsam

