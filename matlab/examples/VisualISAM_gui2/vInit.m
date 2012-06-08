function [ handles ] = vInit( handles )
%VINIT Summary of this function goes here
%   Detailed explanation goes here
%% Initialize iSAM
handles.isam = visualSLAMISAM(handles.REORDER_INTERVAL);
handles.newFactors = visualSLAMGraph;
handles.initialEstimates = visualSLAMValues;
i1 = symbol('x',1);
camera1 = handles.cameras{1};
pose1 = camera1.pose;
if handles.HARD_CONSTRAINT % add hard constraint
    handles.newFactors.addPoseConstraint(i1,pose1);
else
    handles.newFactors.addPosePrior(i1,pose1, handles.poseNoise);
end
handles.initialEstimates.insertPose(i1,pose1);
% Add visual measurement factors from first pose
for j=1:handles.nPoints
    jj = symbol('l',j);
    if handles.POINT_PRIORS % add point priors
        handles.newFactors.addPointPrior(jj, handles.points{j}, handles.pointNoise);
    end
    zij = camera1.project(handles.points{j});
    handles.newFactors.addMeasurement(zij, handles.measurementNoise, i1, jj, handles.K);
    handles.initialEstimates.insertPoint(jj, handles.points{j});
end

handles.frame_i = 1;

end

