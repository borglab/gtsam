%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief A simple visual SLAM example for structure from motion
% @author Duy-Nguyen Ta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Global variables used in VisualISAMExample
global options data isam result frame_i
global poseNoise odometryNoise pointNoise measurementNoise

% Data Options
options.triangle = false;
options.nrCameras = 20;
options.showImages = false;

% iSAM Options
options.hardConstraint = false;
options.pointPriors = false;
options.batchInitialization = true;
options.reorderInterval = 10;
options.alwaysRelinearize = false;

% Display Options
options.saveDotFile = false;
options.printStats = false;
options.drawInterval = 5;
options.cameraInterval = 1;
options.drawTruePoses = false;
options.saveFigures = false;
options.saveDotFiles = false;

%% Generate data
data = VisualISAMGenerateData(options);

%% Initialize iSAM with the first pose and points
VisualISAMInitialize(options)
figure(1);
VisualISAMPlot(data, isam, result, options)

%% Main loop for iSAM: stepping through all poses
for frame_i=3:options.nrCameras
    VisualISAMStep
    if mod(frame_i,options.drawInterval)==0
        VisualISAMPlot(data, isam, result, options)
    end
end