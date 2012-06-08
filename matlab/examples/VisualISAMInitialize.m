function [ isam, results ] = VisualISAMInitialize( data, reorderInterval )
%VISUALISAMINITIALIZE Initialize the first two poses and update ISAM
    if (nargin<2), reorderInterval = 1; end
    isam = visualSLAMISAM(reorderInterval);

    %% Add new factors 
    newFactors = visualSLAMGraph;
    newFactors.addPosePrior(symbol('x',1), data.cameras{1}.pose, data.posePriorNoise);
    newFactors.addPointPrior(symbol('l',1), data.points{1}, data.pointPriorNoise);
    
    odometry = data.cameras{1}.pose().between(data.cameras{2}.pose());
	newFactors.addOdometry(symbol('x',1), symbol('x',2), odometry, data.odometryNoise);

    for i=1:2
        for j=1:size(data.points,2)
            zij = data.cameras{i}.project(data.points{j});
            newFactors.addMeasurement(zij, data.measurementNoise, symbol('x',i), symbol('l',j), data.K);
        end
    end
    
    %% Initial estimats for new variables
    initials = visualSLAMValues;
    initials.insertPose(symbol('x',1), data.cameras{1}.pose);
    initials.insertPose(symbol('x',2), data.cameras{2}.pose);
    for j=1:size(data.points,2)
        initials.insertPoint(symbol('l',j), data.points{j});
    end
    
    %% Update ISAM
    isam.update(newFactors, initials);
    results = isam.estimate();
end

