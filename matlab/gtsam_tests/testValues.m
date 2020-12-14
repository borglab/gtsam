% test wrapping of Values
import gtsam.*;

values = Values;
E = EssentialMatrix(Rot3,Unit3);
tol = 1e-9;

values.insert(0, Point2(0, 0));
values.insert(1, Point3(0, 0, 0));
values.insert(2, Rot2);
values.insert(3, Pose2);
values.insert(4, Rot3);
values.insert(5, Pose3);
values.insert(6, Cal3_S2);
values.insert(7, Cal3DS2);
values.insert(8, Cal3Bundler);
values.insert(9, E);
values.insert(10, imuBias.ConstantBias);

% special cases for Vector and Matrix:
values.insert(11, [1;2;3]);
values.insert(12, [1 2;3 4]);

EXPECT('at',values.atPoint2(0) == Point2(0, 0));
EXPECT('at',values.atPoint3(1) == Point3(0, 0, 0));
EXPECT('at',values.atRot2(2).equals(Rot2,tol));
EXPECT('at',values.atPose2(3).equals(Pose2,tol));
EXPECT('at',values.atRot3(4).equals(Rot3,tol));
EXPECT('at',values.atPose3(5).equals(Pose3,tol));
EXPECT('at',values.atCal3_S2(6).equals(Cal3_S2,tol));
EXPECT('at',values.atCal3DS2(7).equals(Cal3DS2,tol));
EXPECT('at',values.atCal3Bundler(8).equals(Cal3Bundler,tol));
EXPECT('at',values.atEssentialMatrix(9).equals(E,tol));
EXPECT('at',values.atConstantBias(10).equals(imuBias.ConstantBias,tol));

% special cases for Vector and Matrix:
actualVector = values.atVector(11);
EQUALITY('at',[1;2;3],actualVector,tol);
actualMatrix = values.atMatrix(12);
EQUALITY('at',[1 2;3 4],actualMatrix,tol);
