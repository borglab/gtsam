%% data
%data = VisualISAMData_triangle();
data = VisualISAMData_cube();

%% init
[isam results] = VisualISAMInitialize(data);
sprintf('Frame 1,2:')
results
figure(1); clf;
VisualISAMPlot(results, data);

%% Next frame index
frame_i=2; 
%% All steps
while (frame_i<size(data.cameras,2))
    %% one step
    frame_i = frame_i+1;
    [isam results] = VisualISAMStep(frame_i, isam, data, results); 
    results
    figure(1); clf;
    VisualISAMPlot(results, data);
end