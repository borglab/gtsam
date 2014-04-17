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

gtPosition = [xlt, ylt, zlt]';
gtRotation = Rot3; %Rot3.Expmap(rand(3,1)); %Rot3.ypr(gtScenario.Heading(scenarioInd), gtScenario.Pitch(scenarioInd), gtScenario.Roll(scenarioInd));
currentPose = Pose3(gtRotation, Point3(gtPosition));

end