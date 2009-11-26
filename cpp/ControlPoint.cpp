/**
 * @file ControlPoint.cpp
 * @brief Implementation of ControlPoint
 * @author Alex Cunningham
 */

#include <iostream>
#include "ControlPoint.h"

using namespace std;
using namespace gtsam;

ControlPoint::ControlPoint()
: time_(0.0)
{ // Note that default pose2 constructors are at (0,0,0)
}

ControlPoint::ControlPoint(const Pose2& pos, const Pose2& vel, double time)
: pos_(pos), vel_(vel), time_(time)
{
}

ControlPoint::ControlPoint(double posx, double posy, double posr,
						   double velx, double vely, double velr, double time)
: pos_(posx, posy, posr), vel_(velx, vely, velr), time_(time)
{
}

void ControlPoint::print(const std::string& name) const {
	cout << "ControlPoint: " << name << " at time = " << time_ << endl;
	pos_.print("Position");
	vel_.print("Velocity");
}

bool ControlPoint::equals(const ControlPoint& pt, double tol) const {
	bool time_equal = abs(time_-pt.time()) < tol;
	return time_equal && pos_.equals(pt.pos()) && vel_.equals(pt.vel());
}

ControlPoint ControlPoint::exmap(const Vector& delta) {
	//TODO: bound the angle for position to -pi < theta < pi
	Pose2 newPos(pos_.x()+delta(0), pos_.y()+delta(1), pos_.theta()+delta(2));
	Pose2 newVel(vel_.x()+delta(3), vel_.y()+delta(4), vel_.theta()+delta(5));
	double newTime = time_ + delta(6);
	return ControlPoint(newPos, newVel, newTime);
}


