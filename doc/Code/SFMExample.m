%% Add factors for all measurements
noise = noiseModel.Isotropic.Sigma(2, measurementNoiseSigma);
for i = 1:length(Z), 
    for k = 1:length(Z{i})
        j = J{i}{k};
        G.add(GenericProjectionFactorCal3_S2(
              Z{i}{k}, noise, symbol('x', i), symbol('p', j), K));
    end
end
