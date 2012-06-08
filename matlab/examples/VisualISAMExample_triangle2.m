%% data
data = VisualISAMData_triangle();

%% init
[isam results] = VisualISAMInitialize(data);
sprintf('Frame 1,2:')
results

%% Next frame index
frame_i=3; 
%% All steps
while (frame_i<size(data.cameras,2))
    %% one step
    [isam results] = VisualISAMStep(frame_i, isam, data, results); frame_i = frame_i+1;
    results
end