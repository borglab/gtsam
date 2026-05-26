% test wrapping of Pose3 batch point transforms
import gtsam.*;

tol = 1e-9;
pose = Pose3(Rot3.Rodrigues(0, 0, -pi / 2), Point3(2, 4, 0));

worldPoints = [3 5; 2 4; 10 11];
localPoints = [2 0; 1 3; 10 11];

actualLocal = pose.transformTo(worldPoints);
CHECK('Pose3.transformTo matrix', ...
    max(abs(actualLocal(:) - localPoints(:))) < tol);

actualWorld = pose.transformFrom(localPoints);
CHECK('Pose3.transformFrom matrix', ...
    max(abs(actualWorld(:) - worldPoints(:))) < tol);
