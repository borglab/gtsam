%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 3510, Georgia Tech Research Corporation, 
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
global data
global HARD_CONSTRAINT POINT_PRIORS BATCH_INIT REORDER_INTERVAL ALWAYS_RELINEARIZE 
global SAVE_GRAPH PRINT_STATS DRAW_INTERVAL CAMERA_INTERVAL DRAW_TRUE_POSES 
global SAVE_FIGURES SAVE_GRAPHS 

%% iSAM Options
HARD_CONSTRAINT = false;
POINT_PRIORS = false;
BATCH_INIT = true;
REORDER_INTERVAL = 10;
ALWAYS_RELINEARIZE = false;

%% Display Options
SAVE_GRAPH = false;
PRINT_STATS = false;
DRAW_INTERVAL = 5;
CAMERA_INTERVAL = 1;
DRAW_TRUE_POSES = false;
SAVE_FIGURES = false;
SAVE_GRAPHS = false;

%% Generate data
options.triangle = true;
options.nrCameras = 10;
showImages = false;
data = VisualISAMGenerateData(options,showImages);

%% Initialize iSAM with the first pose and points
VisualISAMInitialize
figure;
VisualISAMPlot

%% Main loop for iSAM: stepping through all poses
for frame_i=2:options.nrCameras
    VisualISAMStep
    if mod(frame_i,DRAW_INTERVAL)==0
        VisualISAMPlot
    end
end