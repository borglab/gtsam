/*
 * @file ControlPoint.h
 * @brief Class with a point in a Position-Velocity model at a given time for 2D robots
 * @author Alex Cunningham
 */

#pragma once

#include "Pose2.h"
#include "Testable.h"

namespace gtsam {

/**
 * This class stores a single point in time using a model
 * with position and velocity, as well as a time stamp, and
 * is designed for use with robot control applications.
 */
class ControlPoint : public Testable<ControlPoint> {
private:
	// position model
	Pose2 pos_;

	// velocity model
	Pose2 vel_;

	// timestamp for this observation
	double time_;

public:
	/** default contructor: stationary point at origin at zero time*/
	ControlPoint();

	/** full constructor */
	ControlPoint(const Pose2& pos, const Pose2& vel, double time);

	/** manual constructor - specify each Pose2 in full */
	ControlPoint(double posx, double posy, double posr,
				 double velx, double vely, double velr, double time);

	/** default destructor */
	virtual ~ControlPoint() {}

	/** Standard print function with optional label */
	virtual void print(const std::string& name="") const;

	/** Equality up to a tolerance */
	virtual bool equals(const ControlPoint& expected, double tol=1e-9) const;

	/* Access functions */
	Pose2 pos() const { return pos_; }
	Pose2 vel() const { return vel_; }
	double time() const { return time_; }

	/**
	 * Exmap function to add a delta configuration to the point
	 * NOTE: in handling rotation, the position will have its
	 * range bounded to -pi < r < pi, but the velocity
	 * can be larger than 2pi, as this would represent that
	 * the angular velocity will do more than a full rotation
	 * in a time step.
	 */
	ControlPoint exmap(const Vector& delta);
};
	// comparison

}



