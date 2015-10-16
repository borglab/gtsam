% Test runner script - runs each test 

%% geometry
display 'Starting: testCal3Unified'
testCal3Unified

%% linear
display 'Starting: testKalmanFilter'
testKalmanFilter

display 'Starting: testJacobianFactor'
testJacobianFactor

%% nonlinear
display 'Starting: testValues'
testValues

%% SLAM
display 'Starting: testPriorFactor'
testPriorFactor

%% examples
display 'Starting: testLocalizationExample'
testLocalizationExample

display 'Starting: testOdometryExample'
testOdometryExample

display 'Starting: testPlanarSLAMExample'
testPlanarSLAMExample

display 'Starting: testPose2SLAMExample'
testPose2SLAMExample

display 'Starting: testPose3SLAMExample'
testPose3SLAMExample

display 'Starting: testSFMExample'
testSFMExample

display 'Starting: testStereoVOExample'
testStereoVOExample

display 'Starting: testVisualISAMExample'
testVisualISAMExample

%% MATLAB specific
display 'Starting: testUtilities'
testUtilities

if(exist('testSerialization.m','file'))
    display 'Starting: testSerialization'
    testSerialization
end

% end of tests
display 'Tests complete!'
