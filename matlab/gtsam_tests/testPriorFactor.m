% test wrapping of Values
import gtsam.*;

values = Values;

key = 5; 
priorPose3 = Pose3;
model = noiseModel.Unit.Create(6);
factor = PriorFactorPose3(key, priorPose3, model);
values.insert(key, priorPose3);
EXPECT('error', factor.error(values) == 0);

key = 3; 
priorVector = [0,0,0]';
model = noiseModel.Unit.Create(3);
factor = PriorFactorVector(key, priorVector, model);
values.insert(key, priorVector);
EXPECT('error', factor.error(values) == 0);
