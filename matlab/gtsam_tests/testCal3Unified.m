% test Cal3Unified
K = gtsam.Cal3Unified;
gtsam.EXPECT('fx',K.fx()==1);
gtsam.EXPECT('fy',K.fy()==1);

params = gtsam.PreintegrationParams.MakeSharedU(-9.81);
%params.getOmegaCoriolis()

gtsam.EXPECT('getBodyPSensor default', isempty(params.getBodyPSensor()));
expectedBodyPSensor = gtsam.Pose3(gtsam.Rot3(0, 0, 0, 0, 0, 0, 0, 0, 0), gtsam.Point3(0, 0, 0));
params.setBodyPSensor(expectedBodyPSensor);
gtsam.EXPECT('getBodyPSensor set', expectedBodyPSensor.equals(params.getBodyPSensor(), 1e-9));
