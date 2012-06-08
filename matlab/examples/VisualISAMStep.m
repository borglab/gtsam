function [ isam, results ] = VisualISAMStep( frame_i, isam, data, prevResults )
%VISUALISAMSTEP Summary of this function goes here
%   Detailed explanation goes here

    %% Add new factors 
    newFactors = visualSLAMGraph;
    frame_i
    odometry = data.cameras{frame_i-1}.pose().between(data.cameras{frame_i}.pose());
    newFactors.addOdometry(symbol('x',frame_i-1), symbol('x',frame_i), odometry, data.odometryNoise);
    for j=1:size(data.points,2)
        zij = data.cameras{frame_i}.project(data.points{j});
        newFactors.addMeasurement(zij, data.measurementNoise, symbol('x',frame_i), symbol('l',j), data.K);
    end
    
    %% Initial estimates for new variables
    initials = visualSLAMValues;
    prevPose = prevResults.pose(symbol('x',frame_i-1));
    initials.insertPose(symbol('x',frame_i), prevPose.compose(odometry));
        
    isam.update(newFactors, initials);
    results = isam.estimate();

end

