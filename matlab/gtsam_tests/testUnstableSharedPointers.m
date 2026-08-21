% Regression coverage for the named vector of shared stereo calibrations.

firstCalibration = gtsam.Cal3_S2Stereo(500, 500, 0, 320, 240, 0.2);
secondCalibration = gtsam.Cal3_S2Stereo(510, 505, 0, 320, 240, 0.3);

measurements = gtsam.StereoPoint2Vector();
measurements.push_back(gtsam.StereoPoint2(320, 300, 240));
measurements.push_back(gtsam.StereoPoint2(330, 310, 245));
gtsam.EXPECT('stereo measurement vector size', measurements.size() == 2);
gtsam.EXPECT('stereo measurement vector element', ...
    abs(measurements.at(1).uL() - 330) < 1e-12);

calibrations = gtsam.Cal3_S2StereoVector();
calibrations.push_back(firstCalibration);
calibrations.push_back(secondCalibration);
clear firstCalibration secondCalibration;

gtsam.EXPECT('calibration vector size', calibrations.size() == 2);
storedCalibration = calibrations.at(0);
gtsam.EXPECT('calibration vector element type', ...
    isa(storedCalibration, 'gtsam.Cal3_S2Stereo'));
gtsam.EXPECT('calibration vector element lifetime', ...
    abs(storedCalibration.baseline() - 0.2) < 1e-12);

noise = gtsam.noiseModel.Isotropic.Sigma(3, 1.0);
factor = gtsam.SmartStereoProjectionPoseFactor(noise);
poseKeys = gtsam.KeyVector();
poseKeys.push_back(uint64(1));
poseKeys.push_back(uint64(2));
factor.add(measurements, poseKeys, calibrations);
clear noise measurements poseKeys calibrations storedCalibration;

recovered = factor.calibration();
gtsam.EXPECT('recovered calibration vector type', ...
    isa(recovered, 'gtsam.Cal3_S2StereoVector'));
gtsam.EXPECT('recovered calibration vector size', recovered.size() == 2);
clear factor;
gtsam.EXPECT('recovered first calibration lifetime', ...
    abs(recovered.at(0).baseline() - 0.2) < 1e-12);
gtsam.EXPECT('recovered second calibration lifetime', ...
    abs(recovered.at(1).baseline() - 0.3) < 1e-12);

clear recovered;
