%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create graph container and add factors to it
graph = pose2SLAMGraph;
initial = pose2SLAMValues;
odometryNoise = gtsamSharedNoiseModel_Sigmas([0.2; 0.2; 0.1]);
constraint=gtsamPose2; % identity

%% Add a Gaussian prior on pose x_1
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = gtsamSharedNoiseModel_Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.addPrior(0, priorMean, priorNoise); % add directly to graph

%% Read File and create graph and initial estimate
fid = fopen('../Data/w100-odom.graph');
if fid < 0
    error('Cannot open the file ');
end

columns=textscan(fid,'%s','delimiter','\n');
fclose(fid);
lines=columns{1};
n=size(lines,1);

for i=1:n
    line_i=lines{i};
    if strcmp('VERTEX2',line_i(1:7))
        v = textscan(line_i,'%s %d %f %f %f',1);
        initial.insertPose(v{2}, gtsamPose2(v{3}, v{4}, v{5}));
    elseif strcmp('EDGE2',line_i(1:5))
        e = textscan(line_i,'%s %d %d %f %f %f',1);
        graph.addOdometry(e{2}, e{3}, gtsamPose2(e{4}, e{5}, e{6}), odometryNoise);
    end
end

%% Plot Initial Estimate
figure(1);clf
plot(initial.xs(),initial.ys(),'r-*'); axis equal

addEquivalences=0;
if addEquivalences
    %% Add equivalence constraints
    for i=1:n
        line_i=cell2mat(lines(i));
        if strcmp('EQUIV',line_i(1:5))
            equiv = textscan(line_i,'%s %d %d',1);
            graph.addOdometry(e{2}, e{3}, constraint, odometryNoise);
        end
    end
    
end

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initial);
hold on; plot(result.xs(),result.ys(),'g-*')

%% Plot Covariance Ellipses
% marginals = graph.marginals(result);
% for i=0:result2.size()-1
%     pose_i = result.pose(i);
%     P_i=marginals.marginalCovariance(i);
%     covarianceEllipse([pose_i.x;pose_i.y],P_i,'g')
% end
