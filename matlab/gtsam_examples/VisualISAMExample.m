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

import gtsam.*

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
[data,truth] = VisualISAMGenerateData(options);

%% Initialize iSAM with the first pose and points
[noiseModels,isam,result,nextPose] = VisualISAMInitialize(data,truth,options);
cla;
VisualISAMPlot(truth, data, isam, result, options)

%% Main loop for iSAM: stepping through all poses
for frame_i=3:options.nrCameras
    [isam,result,nextPose] = VisualISAMStep(data,noiseModels,isam,result,truth,nextPose);
    if mod(frame_i,options.drawInterval)==0
        VisualISAMPlot(truth, data, isam, result, options)
    end
end
