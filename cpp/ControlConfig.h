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
	 */
	void addPoint(const std::string& name, const ControlPoint& state);

	/**
	 * returns the path of a particular robot
	 */
	path getPath(const std::string& agentID) const;

	/**
	 * Returns true if agent is in the config
	 */
	bool involvesAgent(const std::string& agentID) const;

	// clearing
	void clear() { paths_.clear(); }
	void clearAgent(const std::string& agentID);
};
} // \namespace gtsam

