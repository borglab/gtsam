%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010-2013, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Monocular VO Example with 5 landmarks and two cameras
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% import
import gtsam.*

%% Create two cameras and corresponding essential matrix E
aRb = Rot3.Yaw(pi/2);
aTb = [.1, 0, 0]';
identity=Pose3;
aPb = Pose3(aRb, aTb);
cameraA = CalibratedCamera(identity);
cameraB = CalibratedCamera(aPb);

%% Create 5 points
P = { [0, 0, 1]', [-0.1, 0, 1]', [0.1, 0, 1]', [0, 0.5, 0.5]', [0, -0.5, 0.5]' };

%% Project points in both cameras
for i=1:5
  pA{i}= cameraA.project(P{i});
  pB{i}= cameraB.project(P{i});
end

%% We start with a factor graph and add epipolar constraints to it
% Noise sigma is 1cm, assuming metric measurements
graph = NonlinearFactorGraph;
model = noiseModel.Isotropic.Sigma(1,0.01);
for i=1:5
  graph.add(EssentialMatrixFactor(1, pA{i}, pB{i}, model));
end

%% Create initial estimate
initial = Values;
trueE = EssentialMatrix(aRb,Unit3(aTb));
initialE = trueE.retract([0.1, -0.1, 0.1, 0.1, -0.1]');
initial.insert(1, initialE);

%% Optimize
parameters = LevenbergMarquardtParams;
% parameters.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, initial, parameters);
result = optimizer.optimize();

%% Print result (as essentialMatrix) and error
error = graph.error(result)
E = result.atEssentialMatrix(1)

