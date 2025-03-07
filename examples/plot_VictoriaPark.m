clear;

gt = dlmread('data/ISAM2_GT_victoriaPark.txt');
gt_pt = dlmread('data/ISAM2_GT_victoriaPark_lm.txt');

% eh_poses = dlmread('build/HybridISAM_victoriaPark.txt');
% eh_points = dlmread('build/HybridISAM_victoriaPark_lm.txt');
eh_poses = dlmread('build/ISAM2_TEST_victoriaPark.txt');
eh_points = dlmread('build/ISAM2_TEST_victoriaPark_lm.txt');

figure;
hold on;
axis equal;
axis([-110 220 -55 250])

plot(gt(:,1), gt(:,2), '-', 'LineWidth', 6, 'color', [0.7 0.9 1.0]);
plot(gt_pt(:,1), gt_pt(:,2), 'o', 'MarkerSize', 4, 'LineWidth', 3, 'color', [0.7 0.9 1.0]);

plot(eh_poses(:,1), eh_poses(:,2), '-', 'LineWidth', 1.5, 'color', [1.0 0.5 1.0]);
plot(eh_points(:,1), eh_points(:,2), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'color', [1.0 0.5 1.0]);

