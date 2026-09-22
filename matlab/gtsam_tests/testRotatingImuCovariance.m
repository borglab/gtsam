% Exercise rotating covariance accessors and factor overloads in the shared wrapper.
function testRotatingImuCovariance
import gtsam.*
initial = NavState(Rot3.RzRyRx(.4, -.3, .6), [2; -3; 1], [.5; 2; -.7]);
bias = imuBias.ConstantBias();
backends = {@PreintegratedImuMeasurements, ...
            @PreintegratedImuMeasurementsManifold, ...
            @PreintegratedImuMeasurementsTangent, ...
            @PreintegratedImuMeasurementsLieGroup, ...
            @PreintegratedImuMeasurementsG, ...
            @PreintegratedCombinedMeasurements, ...
            @PreintegratedCombinedMeasurementsManifold, ...
            @PreintegratedCombinedMeasurementsLieGroup, ...
            @PreintegratedCombinedMeasurementsG};
for rate = {[], zeros(3, 1), [.2; -.3; .4]}
    params = PreintegrationCombinedParams.MakeSharedU(9.81);
    params.setAccelerometerCovariance(.02 * eye(3));
    params.setGyroscopeCovariance(.01 * eye(3));
    params.setIntegrationCovariance(1e-8 * eye(3));
    if ~isempty(rate{1})
        params.setOmegaCoriolis(rate{1});
    end
    for i = 1:numel(backends)
        pim = backends{i}(params, bias);
        for j = 1:8
            pim.integrateMeasurement([.7; -.2; 2], [.3; .2; -.4], .05);
        end
        predicted = pim.predict(initial, bias);
        covariance = pim.residualCovarianceAt(predicted.attitude());
        fallback = pim.predict(NavState(), bias);
        assert(norm(pim.residualCovarianceAt(fallback.attitude()) - ...
                    pim.residualCovariance(), 'fro') < 1e-12);
        assert(isequal(size(covariance), size(pim.preintMeasCov())));
        if isempty(rate{1}) || ~any(rate{1})
            assert(norm(covariance - pim.residualCovariance(), 'fro') < 1e-12);
        else
            assert(norm(covariance - pim.residualCovariance(), 'fro') > 1e-5);
        end
        % Verify each factor's whitening through its scalar error, since the
        % MATLAB noiseModel() accessor returns the base noise-model proxy.
        if ~ismember(i, [1, 5, 6, 9])
            continue
        end
        offset = [.2; -.1; .15; .1; .2; -.3; .2; -.1; .4];
        endpoint = predicted.expmap(offset);
        if i == 1 || i == 5
            if i == 1
                factor = ImuFactor(0, 1, 2, 3, 4, pim, predicted.attitude());
                stateFactor = ImuFactor2(0, 1, 2, pim, predicted.attitude());
            else
                factor = GalileanImuFactor(0, 1, 2, 3, 4, pim, predicted.attitude());
                stateFactor = GalileanImuFactor2(0, 1, 2, pim, predicted.attitude());
            end
            states = Values();
            states.insert(0, initial);
            states.insert(1, endpoint);
            states.insert(2, bias);
            residual = endpoint.logmap(predicted);
            expected = .5 * residual' * (covariance \ residual);
            assert(abs(stateFactor.error(states) - expected) < 1e-7);
        elseif i == 6
            factor = CombinedImuFactor(0, 1, 2, 3, 4, 5, pim, predicted.attitude());
        else
            factor = GalileanCombinedImuFactor(0, 1, 2, 3, 4, 5, pim, predicted.attitude());
        end
        values = Values();
        values.insert(0, initial.pose());
        values.insert(1, initial.velocity());
        values.insert(2, endpoint.pose());
        values.insert(3, endpoint.velocity());
        values.insert(4, bias);
        residual = endpoint.logmap(predicted);
        if i >= 6
            values.insert(5, bias);
            residual = [residual; zeros(6, 1)];
        end
        expected = .5 * residual' * (covariance \ residual);
        assert(abs(factor.error(values) - expected) < 1e-7);
    end
end
fprintf('Rotating IMU covariance wrapper tests passed.\n');
end
