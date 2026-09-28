% Retaining and releasing keys in both concrete fixed-lag smoothers.
% A retained key outlives the lag across later four-argument updates, and is
% marginalized in the same update that releases it.

noise = gtsam.noiseModel.Isotropic.Sigma(2, 0.1);
lag = 2.0;
smoothers = {gtsam.BatchFixedLagSmoother(lag), gtsam.IncrementalFixedLagSmoother(lag)};

for s = 1:numel(smoothers)
  smoother = smoothers{s};
  name = class(smoother);

  % Add X(i) at time i, with a prior for X(0) or odometry from X(i - 1).
  % Retain X(1) in the update that adds it.
  for i = 0:7
    key = gtsam.symbol('x', i);
    factors = gtsam.NonlinearFactorGraph;
    if i == 0
      factors.add(gtsam.PriorFactorPoint2(key, gtsam.Point2(0, 0), noise));
    else
      factors.add(gtsam.BetweenFactorPoint2(gtsam.symbol('x', i - 1), key, gtsam.Point2(1, 0), noise));
    end
    values = gtsam.Values;
    values.insert(key, gtsam.Point2(i, 0));
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap;
    timestamps.insert(gtsam.FixedLagSmootherKeyTimestampMapValue(key, i));
    if i == 1
      keysToRetain = gtsam.KeySet;
      keysToRetain.insert(key);
      smoother.update(factors, values, timestamps, gtsam.FactorIndices, keysToRetain);
    else
      smoother.update(factors, values, timestamps);
    end
  end

  x0 = gtsam.symbol('x', 0);
  x1 = gtsam.symbol('x', 1);
  expectedRetained = gtsam.KeySet;
  expectedRetained.insert(x1);
  gtsam.CHECK([name ' retainedKeys'], smoother.retainedKeys().equals(expectedRetained, 0));
  gtsam.CHECK([name ' retainedKeys again'], smoother.retainedKeys().equals(expectedRetained, 0));
  gtsam.CHECK([name ' retained timestamp'], smoother.timestamps().at(x1) == 1);
  gtsam.CHECK([name ' expired key marginalized'], ~smoother.getLinearizationPoint().exists(x0));
  gtsam.CHECK([name ' retained estimate'], ...
        norm(smoother.calculateEstimatePoint2(x1) - gtsam.Point2(1, 0)) < 1e-6);

  % X(1) is outside the lag, so releasing it marginalizes it now.
  keysToRelease = gtsam.KeySet;
  keysToRelease.insert(x1);
  smoother.update(gtsam.NonlinearFactorGraph, gtsam.Values, gtsam.FixedLagSmootherKeyTimestampMap, ...
                  gtsam.FactorIndices, gtsam.KeySet, keysToRelease);
  gtsam.CHECK([name ' released'], smoother.retainedKeys().empty());
  gtsam.CHECK([name ' released key marginalized'], ~smoother.getLinearizationPoint().exists(x1));
  gtsam.CHECK([name ' newest key kept'], smoother.getLinearizationPoint().exists(gtsam.symbol('x', 7)));
end
