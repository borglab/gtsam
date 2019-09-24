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

clear all;
close all;

x = linspace(-10, 10, 1000);
% x = linspace(-5, 5, 101);

c = 1.3998;
rho = fair(x, c);
fairNoiseModel = gtsam.noiseModel.mEstimator.Fair(c);
plot_m_estimator(x, fairNoiseModel, rho, 'Fair', 1, 'fair.png')

c = 1.345;
rho = huber(x, c);
huberNoiseModel = gtsam.noiseModel.mEstimator.Huber(c);
plot_m_estimator(x, huberNoiseModel, rho, 'Huber', 2, 'huber.png')

c = 0.1;
rho = cauchy(x, c);
cauchyNoiseModel = gtsam.noiseModel.mEstimator.Cauchy(c);
plot_m_estimator(x, cauchyNoiseModel, rho, 'Cauchy', 3, 'cauchy.png')

c = 1.0;
rho = gemanmcclure(x, c);
gemanmcclureNoiseModel = gtsam.noiseModel.mEstimator.GemanMcClure(c);
plot_m_estimator(x, gemanmcclureNoiseModel, rho, 'GemanMcClure', 4, 'gemanmcclure.png')

c = 2.9846;
rho = welsch(x, c);
welschNoiseModel = gtsam.noiseModel.mEstimator.Welsch(c);
plot_m_estimator(x, welschNoiseModel, rho, 'Welsch', 5, 'welsch.png')

c = 4.6851;
rho = tukey(x, c);
tukeyNoiseModel = gtsam.noiseModel.mEstimator.Tukey(c);
plot_m_estimator(x, tukeyNoiseModel, rho, 'Tukey', 6, 'tukey.png')

%% Plot rho, psi and weights of the mEstimator.
function plot_m_estimator(x, model, rho, plot_title, fig_id, filename)
    w = zeros(size(x));
    for i = 1:size(x, 2)
        w(i) = model.weight(x(i));
    end

    psi = w .* x;

    figure(fig_id);
    subplot(3, 1, 1);
    plot(x, rho);
    xlim([-5, 5]);
    title(plot_title);
    subplot(3, 1, 2);
    plot(x, psi);
    xlim([-5, 5]);
    subplot(3, 1, 3);
    plot(x, w);
    xlim([-5, 5]);
    saveas(figure(fig_id), filename);
end

function rho = fair(x, c)
    rho = c^2 * ( (abs(x) ./ c) - log(1 + (abs(x)./c)) );
end

function rho = huber(x, k)
    t = (abs(x) > k);

    rho = (x .^ 2) ./ 2;
    rho(t) = k * (abs(x(t)) - (k/2));
end

function rho = cauchy(x, c)    
    rho = (c^2 / 2) .* log(1 + ((x./c) .^ 2));
end

function rho = gemanmcclure(x, c)
    rho = ((x .^ 2) ./ 2) ./ (1 + x .^ 2);
end

function rho = welsch(x, c)
    rho = (c^2 / 2) * ( 1 - exp(-(x ./ c) .^2 ));
end

function rho = tukey(x, c)
    t = (abs(x) > c);

    rho = (c^2 / 6) * (1 - (1 - (x ./ c) .^ 2 ) .^ 3 );
    rho(t) = c .^ 2 / 6;
end
