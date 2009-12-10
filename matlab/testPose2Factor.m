cov = [ 0.25, 0, 0; 0, 0.25, 0; 0, 0, 0.01];
key1='x1';
key2='x2';
measured=Pose2(2,2, pi/2);
measured.print('Pose');
factor=Pose2Factor(key1,key2,measured, cov);
factor.print('Factor');

p1=Pose2(1.1,2,pi/2); % robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
p2=Pose2(-1,4.1,pi);  % robot at (-1,4) looking at negative (ground truth is at 4.1,2)
config= Pose2Config() ;

%fg = Pose2Graph;