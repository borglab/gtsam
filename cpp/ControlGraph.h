/*
 * @file ControlGraph.h
 * @brief Graph for robot control problems
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <set>
#include "NonlinearFactorGraph.h"
#include "FactorGraph-inl.h"
#include "ControlConfig.h"

namespace gtsam {

/**
 * This graph manages the relationships between time-dependent
 * points in robot trajectories. Each of these points manages its
 * own time, so there will need to be constraints to ensure that
 * the trajectories remain in the correct temporal ordering.
 */
class ControlGraph : public gtsam::NonlinearFactorGraph<ControlConfig>, Testable<ControlGraph> {
public:
	/**
	 * Subclass to handle the model for individual agents
	 */
	class DynamicsModel : Testable<DynamicsModel>{
	private:
		double maxVel_;
		double maxAcc_;
		double maxRotVel_;
		double maxRotAcc_;
	public:

		/** Constructor with unbounded limits */
		DynamicsModel();

		/** Constructor with initialization */
		DynamicsModel(double maxVel, double maxAcc,
				  double maxRotVel, double maxRotAcc);

		virtual ~DynamicsModel() {}

		/** Standard print function with optional label */
		void print(const std::string& name="") const;

		/** Equality up to a tolerance */
		bool equals(const DynamicsModel& expected, double tol=1e-9) const;
	};

public:
	// data typedefs
	typedef std::map<std::string, DynamicsModel>::const_iterator const_model_it;

private:
	/** models for the agents */
	std::map<std::string, DynamicsModel> models_;

	/** feasible set for constraints */
	ControlConfig feasible_;
public:

	/** Default constructor and destructor */
	ControlGraph() {}
	virtual ~ControlGraph() {}

	/** Standard print function with optional label */
	void print(const std::string& name="") const;

	/** Equality up to a tolerance */
	bool equals(const ControlGraph& expected, double tol=1e-9) const;

	/**
	 * Adds an agent with parameters for the robot itself
	 * @param name is the name of the agent
	 * @param maxVel is the maximum translational velocity in distance/time
	 * @param maxAcc is the maximum translational acceleration in velocity/time
	 * @param maxRotVel is the maximum rotational velocity in radians/time
	 * @param maxRotAcc is the maximum rotational acceleration in anglar velocity/time
	 */
	void addAgent(const std::string& name,
				  double maxVel, double maxAcc,
				  double maxRotVel, double maxRotAcc);

	/**
	 * Adds an agent with particular model
	 * @param name is the name of the agent
	 * @param model defines the characteristics of the robot
	 */
	void addAgent(const std::string& name, const DynamicsModel& model);

	/** number of agents */
	size_t nrAgents() const { return models_.size(); }

	/** list of agents */
	std::set<std::string> agents() const;

	/**
	 * Gets the dynamics model for an agent
	 */
	DynamicsModel agentModel(const std::string& agent) const;

	/**
	 * Creates a trajectory segment for a robot and adds it to the
	 * end of the existing path for given robot
	 * @param name is the name of the agent
	 * @param states is the number of additional states after the initial state
	 */
	void addTrajectory(const std::string& name, size_t states);

	/**
	 * Fixes a particular state in a trajectory to a given point using
	 * a NonlinearEquality constraint.  Use this for setting start and
	 * end points of trajectories.
	 * @param name is the name of the agent
	 * @param state is the value to fix the state to
	 * @param state_num is the number of the state to fix (defaults to first state)
	 */
	void fixAgentState(const std::string& name, const ControlPoint& state, size_t state_num=0);

	/**
	 * Returns the feasible set for all of the constraints currently constructed
	 * @return config with constrained values
	 * NOTE: this will pad trajectories with default states
	 */
	ControlConfig feasible() const { return feasible_; }

};

} // \namespace gtsam


