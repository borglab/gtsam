% test wrapping of Pose3 batch point transforms
tol = 1e-9;
pose = gtsam.Pose3(gtsam.Rot3.Rodrigues(0, 0, -pi / 2), ...
                   gtsam.Point3(2, 4, 0));

worldPoints = [3 5; 2 4; 10 11];
localPoints = [2 0; 1 3; 10 11];

actualLocal = pose.transformTo(worldPoints);
gtsam.CHECK('Pose3.transformTo matrix', ...
    max(abs(actualLocal(:) - localPoints(:))) < tol);

actualWorld = pose.transformFrom(localPoints);
gtsam.CHECK('Pose3.transformFrom matrix', ...
    max(abs(actualWorld(:) - worldPoints(:))) < tol);

worldSquare = [0 0 1 1; 0 1 1 0; 0 0 0 0];
localSquare = pose.transformTo(worldSquare);
alignedPose = gtsam.Pose3.Align(worldSquare, localSquare);
gtsam.CHECK('Pose3.Align matrix', alignedPose.equals(pose, tol));
