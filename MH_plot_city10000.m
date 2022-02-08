clear;

gt = dlmread('data/ISAM2_GT_city10000.txt');

eh_poses = dlmread('build/MH_ISAM2_TEST_city10000_hypos.txt');

figure;
hold on;
axis equal;
axis([-65 65 -75 60])

plot(gt(:,1), gt(:,2), '-', 'LineWidth', 4, 'color', [0.7 0.7 0.7]);

h_num = size(eh_poses, 1)

% Plot all hypos
for inv_h = 1:h_num
    h = h_num - inv_h + 1;
    plot(eh_poses(h,1:2:end), eh_poses(h,2:2:end), '-', 'LineWidth', 2, 'color', [(h/h_num) (1-h/h_num)/2 1.0]);
end

