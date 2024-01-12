% test Cal3Unified
import gtsam.*;

K = Cal3Unified;
EXPECT('fx',K.fx()==1);
EXPECT('fy',K.fy()==1);

params = PreintegrationParams.MakeSharedU(-9.81);
%params.getOmegaCoriolis()

expectedBodyPSensor = gtsam.Pose3(gtsam.Rot3(0, 0, 0, 0, 0, 0, 0, 0, 0), gtsam.Point3(0, 0, 0));
EXPECT('getBodyPSensor', expectedBodyPSensor.equals(params.getBodyPSensor(), 1e-9));
