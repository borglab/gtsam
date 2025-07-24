% for instructions, see https://github.com/borglab/gtsam/blob/develop/matlab/README.md
% don't know how to set up paths properly yet... seems like it only needs gtsamDebug
% addpath('~/gtsam/')
addpath('~/gtsamDebug/')
format compact
% addpath('~/gtsamDebug/gtsam_examples/')

%% compare with gt
load /home/daniel/Dropbox/Coursework/year_1/AER1513/assignment/3/dataset3.mat
k1 = 1215;
k2 = 1714-1;
n = k2 - k1+1;
r_gt = r_i_vk_i(:,k1:k2);
theta_gt = theta_vk_i(:,k1:k2);
C_gt = zeros(3,3,n);
for i = 1:n
    C_gt(:,:,i) = psitoC(theta_gt(:,i));
end
%%
[graph_init, initial] = gtsam.readG2o('/home/daniel/gtsam/gtsam_repo/build/starry_night_initial.g2o', true);
[graph_opt, optimized] = gtsam.readG2o('/home/daniel/gtsam/gtsam_repo/build/starry_night_optimized.g2o', true);

%% get solution poses
C_array = zeros(3,3,n);
t_array = zeros(3,n);
for i = 1:n
    pose3 = optimized.atPose3(i);
    C_array(:,:,i) = pose3.rotation().matrix()';
    t_array(:,i) = pose3.translation();
end

% get dr poses
C_array_dr = zeros(3,3,n);
t_array_dr = zeros(3,n);
for i = 1:n
    pose3_dr = initial.atPose3(i);
    C_array_dr(:,:,i) = pose3_dr.rotation().matrix()';
    t_array_dr(:,i) = pose3_dr.translation();
end

% get landmarks - just for double checking. They shouldn't change compared
% to the gt
m = 20;  % num of landmarks
l_array = zeros(3,m);
for j = 1:m
    l_array(:,j) = optimized.atPoint3(gtsam.symbol('l', j-1));
end

%% get marginals
marg_input = readmatrix('/home/daniel/gtsam/gtsam_repo/build/starry_night_marginals.txt');
% each row is a flattened cov matrix
P = reshape(marg_input', 6, 6, []);
sigmas_3 = zeros(6, n);
for i = 1:n
    sigmas_3(:,i) = 3*sqrt(diag(P(:,:,i)));
end
%% plotting
figure
gtsam.plot3DPoints(initial, 'oblack');  % Red stars for landmarks
gtsam.plot3DTrajectory(initial, 'g');      % Green for initial guess
gtsam.plot3DTrajectory(optimized, 'b');      % Blue for optimized
hold on
grid on;
view(3);
axis equal;
xlabel('x'); ylabel('y'); zlabel('z')
legend

%% plotting, same format as a3.m
figure
scatter3(l_array(1,:), l_array(2,:), l_array(3,:), 'DisplayName', 'Landmarks')
hold on
plot3(t_array(1,:), t_array(2,:), t_array(3,:), 'DisplayName', 'Gauss-Newton')  % solution from Gauss-Newton
plot3(r_gt(1,:), r_gt(2,:), r_gt(3,:), 'DisplayName', 'Groundtruth')  % ground truth
plot3(t_array_dr(1,:), t_array_dr(2,:), t_array_dr(3,:), 'DisplayName', 'Dead reckoning')  % dead reckoning
xlabel('x'); ylabel('y'); zlabel('z')
axis equal
legend
title('A3 with GTSAM')
%% errors
error_r = t_array - r_gt;
error_theta = zeros(3, n);
for k = 1:n
    dC = eye(3) - C_array(:,:,k) * C_gt(:,:,k)';
    error_theta(:,k) = [-dC(2,3); dC(1,3); -dC(1,2)];
end

%% plot errors
lim_trans = 0.15;
lim_rot = 0.3;
figure
subplot(3,1,1)
title('x error')
xlabel('time (s)')
ylabel('error in x (m)')
hold on
plot(t(k1:k2), error_r(1,:));
plot(t(k1:k2), sigmas_3(1,:), '--');
plot(t(k1:k2), -sigmas_3(1,:), '--');
ylim([-lim_trans lim_trans])

subplot(3,1,2)
title('y error')
xlabel('time (s)')
ylabel('error in y (m)')
hold on
plot(t(k1:k2), error_r(2,:));
plot(t(k1:k2), sigmas_3(2,:), '--');
plot(t(k1:k2), -sigmas_3(2,:), '--');
ylim([-lim_trans lim_trans])

subplot(3,1,3)
title('z error')
xlabel('time (s)')
ylabel('error in z (m)')
hold on
plot(t(k1:k2), error_r(3,:));
plot(t(k1:k2), sigmas_3(3,:), '--');
plot(t(k1:k2), -sigmas_3(3,:), '--');
ylim([-lim_trans lim_trans])

figure
subplot(3,1,1)
title('\theta_x error')
xlabel('time (s)')
ylabel('error in \theta_x (rad)')
hold on
plot(t(k1:k2), error_theta(1,:));
plot(t(k1:k2), sigmas_3(4,:), '--');
plot(t(k1:k2), -sigmas_3(4,:), '--');
ylim([-lim_rot lim_rot])

subplot(3,1,2)
title('\theta_y error')
xlabel('time (s)')
ylabel('error in \theta_y (rad)')
hold on
plot(t(k1:k2), error_theta(2,:));
plot(t(k1:k2), sigmas_3(5,:), '--');
plot(t(k1:k2), -sigmas_3(5,:), '--');
ylim([-lim_rot lim_rot])

subplot(3,1,3)
title('\theta_z error')
xlabel('time (s)')
ylabel('error in \theta_z (rad)')
hold on
plot(t(k1:k2), error_theta(3,:));
plot(t(k1:k2), sigmas_3(6,:), '--');
plot(t(k1:k2), -sigmas_3(6,:), '--');
ylim([-lim_rot lim_rot])

%%
function C = psitoC(psi)

validateattributes(psi,{'double'},{'nrows',3})

psinorm = norm(psi);
C = cos(psinorm) * eye(3) + (1-cos(psinorm)) * (psi/psinorm) * (psi/psinorm)' - sin(psinorm)*hat(psi/psinorm);

end