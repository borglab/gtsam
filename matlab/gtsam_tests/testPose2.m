% test wrapping of Pose2 batch point transforms
tol = 1e-9;
pose = gtsam.Pose2(2, 4, -pi / 2);

worldPoints = [3 5; 2 4];
localPoints = [2 0; 1 3];

actualLocal = pose.transformTo(worldPoints);
gtsam.CHECK('Pose2.transformTo matrix', ...
    max(abs(actualLocal(:) - localPoints(:))) < tol);

actualWorld = pose.transformFrom(localPoints);
gtsam.CHECK('Pose2.transformFrom matrix', ...
    max(abs(actualWorld(:) - worldPoints(:))) < tol);

alignedPose = gtsam.Pose2.Align(worldPoints, localPoints);
gtsam.CHECK('Pose2.Align matrix', alignedPose.equals(pose, tol));
