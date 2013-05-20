function [ finalPose, finalVelocityGlobal ] = integrateTrajectory( ...
    initialPose, omega1Body, velocity1Body, velocity2Body, deltaT)
%INTEGRATETRAJECTORY Integrate one trajectory step

import gtsam.*;

body2in1 = Rot3.Expmap(omega1Body * deltaT);
velocity2inertial = body2in1.rotate(Point3(velocity2Body)).vector;
accelBody1 = (velocity2inertial - velocity1Body) / deltaT;

initialVelocityGlobal = initialPose.rotation().rotate(Point3(velocity1Body)).vector;
accelGlobal = initialPose.rotation().rotate(Point3(accelBody1)).vector;

finalPosition = Point3(initialPose.translation.vector + initialVelocityGlobal * deltaT + 0.5 * accelGlobal * deltaT * deltaT);
finalVelocityGlobal = initialVelocityGlobal + accelGlobal * deltaT;
finalRotation = initialPose.rotation.compose(body2in1);
finalPose = Pose3(finalRotation, finalPosition);

end

