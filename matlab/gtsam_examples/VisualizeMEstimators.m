%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Plot visualizations of residuals, residual derivatives, and weights for the various mEstimators.
% @author Varun Agrawal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% import gtsam.*

close all;

x = linspace(-10, 10, 1000);

%% Define all the mEstimator models and plot them

c = 1.3998;
fairNoiseModel = gtsam.noiseModel.mEstimator.Fair(c);
plot_m_estimator(x, fairNoiseModel, 'Fair', 1, 'fair.png')

c = 1.345;
huberNoiseModel = gtsam.noiseModel.mEstimator.Huber(c);
plot_m_estimator(x, huberNoiseModel, 'Huber', 2, 'huber.png')

c = 0.1;
cauchyNoiseModel = gtsam.noiseModel.mEstimator.Cauchy(c);
plot_m_estimator(x, cauchyNoiseModel, 'Cauchy', 3, 'cauchy.png')

c = 1.0;
gemanmcclureNoiseModel = gtsam.noiseModel.mEstimator.GemanMcClure(c);
plot_m_estimator(x, gemanmcclureNoiseModel, 'Geman-McClure', 4, 'gemanmcclure.png')

c = 2.9846;
welschNoiseModel = gtsam.noiseModel.mEstimator.Welsch(c);
plot_m_estimator(x, welschNoiseModel, 'Welsch', 5, 'welsch.png')

c = 4.6851;
tukeyNoiseModel = gtsam.noiseModel.mEstimator.Tukey(c);
plot_m_estimator(x, tukeyNoiseModel, 'Tukey', 6, 'tukey.png')

%% Plot rho, psi and weights of the mEstimator.
function plot_m_estimator(x, model, plot_title, fig_id, filename)
    w = zeros(size(x));
    rho = zeros(size(x));
    for i = 1:size(x, 2)
        w(i) = model.weight(x(i));
        rho(i) = model.loss(x(i));
    end

    psi = w .* x;

    figure(fig_id);
    subplot(3, 1, 1);
    plot(x, rho, 'LineWidth',2);
    title('rho function');
    xlim([-5, 5]);
    subplot(3, 1, 2);
    plot(x, psi, 'LineWidth',2);
    title('influence function');
    xlim([-5, 5]);
    subplot(3, 1, 3);
    plot(x, w, 'LineWidth',2);
    title('weight function');
    xlim([-5, 5]);

    sgtitle(plot_title, 'FontSize', 26);

    saveas(figure(fig_id), filename);
end
