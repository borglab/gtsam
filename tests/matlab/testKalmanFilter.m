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
modelQ = SharedDiagonal([0.1;0.1]);
Q = 0.01*eye(2,2);
H = eye(2,2);
z1 = [1.0, 0.0]';
z2 = [2.0, 0.0]';
z3 = [3.0, 0.0]';
modelR = SharedDiagonal([0.1;0.1]);
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

%% Create the Kalman Filter initialization point
x_initial = [0.0;0.0];
P_initial = SharedDiagonal([0.1;0.1]);

%% Create an KalmanFilter object
kalmanFilter = KalmanFilter(x_initial, P_initial)
EQUALITY('expected0,kalmanFilter.mean', expected0,kalmanFilter.mean);
EQUALITY('expected0,kalmanFilter.mean', P00,kalmanFilter.covariance);

%% Run iteration 1
kalmanFilter.predict(F, B, u, modelQ);
EQUALITY('expected1,kalmanFilter.mean', expected1,kalmanFilter.mean);
EQUALITY('P01,kalmanFilter.covariance', P01,kalmanFilter.covariance);
kalmanFilter.update(H,z1,modelR);
EQUALITY('expected1,kalmanFilter.mean', expected1,kalmanFilter.mean);
EQUALITY('I11,kalmanFilter.information', I11,kalmanFilter.information);

%% Run iteration 2
kalmanFilter.predict(F, B, u, modelQ);
EQUALITY('expected2,kalmanFilter.mean', expected2,kalmanFilter.mean);
kalmanFilter.update(H,z2,modelR);
EQUALITY('expected2,kalmanFilter.mean', expected2,kalmanFilter.mean);

%% Run iteration 3
kalmanFilter.predict(F, B, u, modelQ);
EQUALITY('expected3,kalmanFilter.mean', expected3,kalmanFilter.mean);
kalmanFilter.update(H,z3,modelR);
EQUALITY('expected3,kalmanFilter.mean', expected3,kalmanFilter.mean);
