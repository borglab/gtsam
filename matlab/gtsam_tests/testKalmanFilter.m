% /* ----------------------------------------------------------------------------
%
%  * GTSAM Copyright 2010, Georgia Tech Research Corporation,
%  * Atlanta, Georgia 30332-0415
%  * All Rights Reserved
%  * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
%  * See LICENSE for the license information
%
%  * -------------------------------------------------------------------------- */
%
% /**
%  * @file testKalmanFilter.cpp
%  * @brief Test simple linear Kalman filter on a moving 2D point
%  * @date Sep 3, 2011
%  * @author Stephen Williams
%  * @author Frank Dellaert
%  * @author Richard Roberts
%  */

%% Create the controls and measurement properties for our example
F = eye(2,2);
B = eye(2,2);
u = [1.0; 0.0];
modelQ = gtsam.noiseModel.Diagonal.Sigmas([0.1;0.1]);
Q = 0.01*eye(2,2);
H = eye(2,2);
z1 = [1.0, 0.0]';
z2 = [2.0, 0.0]';
z3 = [3.0, 0.0]';
modelR = gtsam.noiseModel.Diagonal.Sigmas([0.1;0.1]);
R = 0.01*eye(2,2);

%% Create the set of expected output TestValues
expected0 = [0.0, 0.0]';
P00 = 0.01*eye(2,2);

expected1 = [1.0, 0.0]';
P01 = P00 + Q;
I11 = inv(P01) + inv(R);

expected2 = [2.0, 0.0]';
P12 = inv(I11) + Q;
I22 = inv(P12) + inv(R);

expected3 = [3.0, 0.0]';
P23 = inv(I22) + Q;
I33 = inv(P23) + inv(R);

%% Create an KalmanFilter object
KF = gtsam.KalmanFilter(2);

%% Create the Kalman Filter initialization point
x_initial = [0.0;0.0];
P_initial = 0.01*eye(2);

%% Create an KF object
state = KF.init(x_initial, P_initial);
gtsam.EQUALITY('expected0,state.mean', expected0,state.mean);
gtsam.EQUALITY('expected0,state.mean', P00,state.covariance);

%% Run iteration 1
state = KF.predict(state,F, B, u, modelQ);
gtsam.EQUALITY('expected1,state.mean', expected1,state.mean);
gtsam.EQUALITY('P01,state.covariance', P01,state.covariance);
state = KF.update(state,H,z1,modelR);
gtsam.EQUALITY('expected1,state.mean', expected1,state.mean);
gtsam.EQUALITY('I11,state.information', I11,state.information);

%% Run iteration 2
state = KF.predict(state,F, B, u, modelQ);
gtsam.EQUALITY('expected2,state.mean', expected2,state.mean);
state = KF.update(state,H,z2,modelR);
gtsam.EQUALITY('expected2,state.mean', expected2,state.mean);

%% Run iteration 3
state = KF.predict(state,F, B, u, modelQ);
gtsam.EQUALITY('expected3,state.mean', expected3,state.mean);
state = KF.update(state,H,z3,modelR);
gtsam.EQUALITY('expected3,state.mean', expected3,state.mean);
