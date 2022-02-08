clear;

gt = dlmread('data/ISAM2_GT_victoriaPark.txt');
gt_pt = dlmread('data/ISAM2_GT_victoriaPark_lm.txt');

eh_poses = dlmread('build/MH_ISAM2_TEST_victoriaPark_hypos.txt');
eh_points = dlmread('build/MH_ISAM2_TEST_victoriaPark_lm_hypos.txt');

figure;
hold on;
axis equal;
axis([-110 220 -55 250])

plot(gt(:,1), gt(:,2), '-', 'LineWidth', 6, 'color', [0.7 0.9 1.0]);
plot(gt_pt(:,1), gt_pt(:,2), 'o', 'MarkerSize', 4, 'LineWidth', 3, 'color', [0.7 0.9 1.0]);

h_num = size(eh_poses, 1);

% Plot all hypos
for inv_h = 1:h_num
    h = h_num - inv_h + 1;
    plot(eh_poses(h,1:2:end), eh_poses(h,2:2:end), '-', 'LineWidth', 1.5, 'color', [(h/h_num) (1-h/h_num)/2 1.0]);
    plot(eh_points(h,1:2:end), eh_points(h,2:2:end), 'o', 'MarkerSize', 3, 'LineWidth', 1, 'color', [(h/h_num) (1-h/h_num)/2 1.0]);
end
