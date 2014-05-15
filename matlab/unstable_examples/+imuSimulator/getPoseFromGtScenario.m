function currentPose = getPoseFromGtScenario(gtScenario,scenarioInd)
% gtScenario contains vectors (Lat, Lon, Alt, Roll, Pitch, Yaw)
% The function picks the index 'scenarioInd' in those vectors and 
% computes the corresponding pose by
% 1) Converting (Lat,Lon,Alt) to local coordinates
% 2) Converting (Roll,Pitch,Yaw) to a rotation matrix

import gtsam.*;

Org_lat = gtScenario.Lat(1);
Org_lon = gtScenario.Lon(1);
initialPositionECEF = imuSimulator.LatLonHRad_to_ECEF([gtScenario.Lat(1); gtScenario.Lon(1); gtScenario.Alt(1)]);

gtECEF = imuSimulator.LatLonHRad_to_ECEF([gtScenario.Lat(scenarioInd); gtScenario.Lon(scenarioInd); gtScenario.Alt(scenarioInd)]);
% truth in ENU
dX = gtECEF(1) - initialPositionECEF(1);
dY = gtECEF(2) - initialPositionECEF(2);
dZ = gtECEF(3) - initialPositionECEF(3);
[xlt, ylt, zlt] = imuSimulator.ct2ENU(dX, dY, dZ,Org_lat, Org_lon);

gtPosition = Point3([xlt, ylt, zlt]');
% use the gtsam.Rot3.Ypr constructor (yaw, pitch, roll) from the ground truth data
%   yaw = measured positively to the right
%   pitch = measured positively up
%   roll = measured positively to right
% Assumes vehice X forward, Y right, Z down
%
% In the gtScenario data
%   heading (yaw) = measured positively to the left from Y-axis
%   pitch = 
%   roll = 
% Coordinate frame is Y forward, X is right, Z is up
gtBodyRotation = Rot3.Ypr(-gtScenario.Heading(scenarioInd), gtScenario.Pitch(scenarioInd), gtScenario.Roll(scenarioInd));
currentPose = Pose3(gtBodyRotation, gtPosition);

%% Rotate the pose
currentPose = currentPose.compose(Pose3.Expmap([-pi/2;0;0;0;0;0]));

end