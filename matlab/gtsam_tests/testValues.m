% test wrapping of Values
values = gtsam.Values;
E = gtsam.EssentialMatrix(gtsam.Rot3, gtsam.Unit3);
tol = 1e-9;

values.insert(0, gtsam.Point2(0, 0));
values.insert(1, gtsam.Point3(0, 0, 0));
values.insert(2, gtsam.Rot2);
values.insert(3, gtsam.Pose2);
values.insert(4, gtsam.Rot3);
values.insert(5, gtsam.Pose3);
values.insert(6, gtsam.Cal3_S2);
values.insert(7, gtsam.Cal3DS2);
values.insert(8, gtsam.Cal3Bundler);
values.insert(9, E);
values.insert(10, gtsam.imuBias.ConstantBias);

% special cases for Vector and Matrix:
values.insert(11, [1;2;3]);
values.insert(12, [1 2;3 4]);

gtsam.EXPECT('at',values.atPoint2(0) == gtsam.Point2(0, 0));
gtsam.EXPECT('at',values.atPoint3(1) == gtsam.Point3(0, 0, 0));
gtsam.EXPECT('at',values.atRot2(2).equals(gtsam.Rot2,tol));
gtsam.EXPECT('at',values.atPose2(3).equals(gtsam.Pose2,tol));
gtsam.EXPECT('at',values.atRot3(4).equals(gtsam.Rot3,tol));
gtsam.EXPECT('at',values.atPose3(5).equals(gtsam.Pose3,tol));
gtsam.EXPECT('at',values.atCal3_S2(6).equals(gtsam.Cal3_S2,tol));
gtsam.EXPECT('at',values.atCal3DS2(7).equals(gtsam.Cal3DS2,tol));
gtsam.EXPECT('at',values.atCal3Bundler(8).equals(gtsam.Cal3Bundler,tol));
gtsam.EXPECT('at',values.atEssentialMatrix(9).equals(E,tol));
gtsam.EXPECT('at',values.atConstantBias(10).equals(gtsam.imuBias.ConstantBias,tol));

% special cases for Vector and Matrix:
actualVector = values.atVector(11);
gtsam.EQUALITY('at',[1;2;3],actualVector,tol);
actualMatrix = values.atMatrix(12);
gtsam.EQUALITY('at',[1 2;3 4],actualMatrix,tol);
