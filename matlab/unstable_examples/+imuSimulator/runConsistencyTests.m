% use this script to easily run and save results for multiple consistency
% tests without having to pay attention to the computer every 5 minutes

import gtsam.*;

resultsDir = 'results/'
if (~exist(resultsDir, 'dir'))
    mkdir(resultsDir);
end

testOptions = [ ...
    %    1       2       3       4       5       6       7     8      9        10         11      12
    % RealData? Between? IMU? IMUType  Bias?  Camera? #LndMrk GPS? StrtPose TrajLength Subsample #MCRuns
         %1        0       1      2       0       0      100    0     100       209          20       100   ;... % 1
         %1        0       1      2       0       0      100    0     100       209          20       100   ;... % 2
        % 1        0       1      2       0       0      100    0     100       209          20       100   ;... % 3
         1        0       1      2       0       1      100    0     100       209          20       20   ;... % 4
         1        0       1      2       0       1      100    0     100       209          20       20   ;... % 5
         1        0       1      2       0       0      100    0     100       209          20       20   ];%... % 6
        % 1        0       1      2       0       0      100    0     100       209          20       100   ;... % 7
         %1        0       1      2       0       0      100    0     100       209          20       1   ;... % 8
         %1        0       1      2       0       0      100    0     100       209          20       1   ];   % 9
   
noises = [ ...
    %      1         2          3           4          5               6              7         8
    % sigma_ang sigma_cart sigma_accel sigma_gyro sigma_accelBias sigma_gyroBias sigma_gps sigma_camera
         %1e-2      1e-1       1e-3        1e-5         0               0            1e-4       1;... % 1
         %1e-2      1e-1       1e-2        1e-5         0               0            1e-4       1;... % 2
        % 1e-2      1e-1       1e-1        1e-5         0               0            1e-4       1;... % 3
         1e-2      1e-1       1e-3        1e-4         0               0            1e-4        1;... % 4
         1e-2      1e-1       1e-3        1e-3         0               0            1e-4        1;... % 5
         1e-2      1e-1       1e-3        1e-2         0               0            1e-4        1];%... % 6
        % 1e-2      1e-1       1e-3        1e-1         0               0            1e-4       1;... % 7
         %1e-2      1e-1       1e-3        1e-2         1e-3            1e-5         1e-4       1;... % 8
         %1e-2      1e-1       1e-3        1e-2         1e-4            1e-6         1e-4       1];   % 9
     
if(size(testOptions,1) ~= size(noises,1))
  error('testOptions and noises do not have same number of rows');
end

% Set flag so the script knows there is an external configuration
externallyConfigured = 1;

% Set the flag to save the results
saveResults = 0;

errorRuns = [];

% Go through tests
for i = 1:size(testOptions,1)
    % Clean up from last test
    close all;
    %clc;
    
    % Set up variables for test
    options.useRealData = testOptions(i,1);
    options.includeBetweenFactors = testOptions(i,2);
    options.includeIMUFactors = testOptions(i,3);
    options.imuFactorType = testOptions(i,4);
    options.imuNonzeroBias = testOptions(i,5);
    options.includeCameraFactors = testOptions(i,6);
    options.numberOfLandmarks = testOptions(i,7);
    options.includeGPSFactors = testOptions(i,8);
    options.gpsStartPose = testOptions(i,9);
    options.trajectoryLength = testOptions(i,10);
    options.subsampleStep = testOptions(i,11);
    numMonteCarloRuns = testOptions(i,12);
    
    sigma_ang = noises(i,1);
    sigma_cart = noises(i,2);
    sigma_accel = noises(i,3);
    sigma_gyro = noises(i,4);
    sigma_accelBias = noises(i,5);
    sigma_gyroBias = noises(i,6);
    sigma_gps = noises(i,7);
    sigma_camera = noises(i,8);
    
    % Create folder name
    f_between = '';
    f_imu = '';
    f_bias = '';
    f_gps = '';
    f_camera = '';
    f_runs = '';
    
    if (options.includeBetweenFactors == 1);
        f_between = 'between_';
    end
    if (options.includeIMUFactors == 1)
        f_imu = sprintf('imu%d_', options.imuFactorType);
        if (options.imuNonzeroBias == 1);
            f_bias = sprintf('bias_a%1.2g_g%1.2g_', sigma_accelBias, sigma_gyroBias);
        end
    end
    if (options.includeGPSFactors == 1);
        f_between = sprintf('gps_%d_', gpsStartPose);
    end
    if (options.includeCameraFactors == 1)
        f_camera = sprintf('camera_%d_', options.numberOfLandmarks);
    end
    f_runs = sprintf('mc%d', numMonteCarloRuns);
    
    folderName = [resultsDir f_between f_imu f_bias f_gps f_camera f_runs '/'];
    
    % make folder if it doesnt exist
    if (~exist(folderName, 'dir'))
        mkdir(folderName);
    end
    
    testName = sprintf('sa-%1.2g-sc-%1.2g-sacc-%1.2g-sg-%1.2g',sigma_ang,sigma_cart,sigma_accel,sigma_gyro);

    % Run the test
    fprintf('Test %d\n\tResults will be saved to:\n\t%s\n\trunning...\n', i, folderName);
    fprintf('Test Name: %s\n', testName);
    
    try
        imuSimulator.covarianceAnalysisBetween;
    catch
        errorRuns = [errorRuns i];
        fprintf('\n*****\n   Something went wrong, most likely indeterminant linear system error.\n');
        disp('Test Options:\n');
        disp(testOptions(i,:));
        disp('Noises');
        disp(noises(i,:));
        fprintf('\n*****\n\n');
    end
end

% Print error summary
fprintf('*************************\n');
fprintf('%d Runs failed due to errors (data not collected for failed runs)\n', length(errorRuns));
for i = 1:length(errorRuns)
    k = errorRuns(i);
    fprintf('\nTest %d:\n', k);
    fprintf('  options.useRealData = %d\n', testOptions(k,1));
    fprintf('  options.includeBetweenFactors = %d\n', testOptions(k,2));
    fprintf('  options.includeIMUFactors = %d\n', testOptions(k,3));
    fprintf('  options.imuFactorType = %d\n', testOptions(k,4));
    fprintf('  options.imuNonzeroBias = %d\n', testOptions(k,5));
    fprintf('  options.includeCameraFactors = %d\n', testOptions(k,6));
    fprintf('  numberOfLandmarks = %d\n', testOptions(k,7));
    fprintf('  options.includeGPSFactors = %d\n', testOptions(k,8));
    fprintf('  options.gpsStartPose = %d\n', testOptions(k,9));
    fprintf('  options.trajectoryLength = %d\n', testOptions(k,10));
    fprintf('  options.subsampleStep = %d\n', testOptions(k,11));
    fprintf('  numMonteCarloRuns = %d\n', testOptions(k,12));
    fprintf('\n');
    fprintf('  sigma_ang = %f\n', noises(i,1));
    fprintf('  sigma_cart = %f\n', noises(i,2));
    fprintf('  sigma_accel = %f\n', noises(i,3));
    fprintf('  sigma_gyro = %f\n', noises(i,4));
    fprintf('  sigma_accelBias = %f\n', noises(i,5));
    fprintf('  sigma_gyroBias = %f\n', noises(i,6));
    fprintf('  sigma_gps = %f\n', noises(i,7));
end


















