% VisualISAMDemo: runs VisualSLAM iSAM demo in a GUI
% Authors: Duy Nguyen Ta

% Make sure global variables are visible on command prompt
% so you can examine how they change as you step through
global data
global poseNoise pointNoise odometryNoise measurementNoise
global frame_i isam result

% Start GUI
VisualISAM_gui