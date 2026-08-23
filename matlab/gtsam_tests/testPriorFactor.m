% test wrapping of Values
values = gtsam.Values;

key = 5; 
priorPose3 = gtsam.Pose3;
model = gtsam.noiseModel.Unit.Create(6);
factor = gtsam.PriorFactorPose3(key, priorPose3, model);
values.insert(key, priorPose3);
gtsam.EXPECT('error', factor.error(values) == 0);

key = 3; 
priorVector = [0,0,0]';
model = gtsam.noiseModel.Unit.Create(3);
factor = gtsam.PriorFactorVector(key, priorVector, model);
values.insert(key, priorVector);
gtsam.EXPECT('error', factor.error(values) == 0);
