% Exercise the planning models, graph controllers and fixed-size Values types.
% Authors: Akash Ratheesh and Frank Dellaert. See LICENSE for details.

function testPlanning()
sampler = gtsam.GaussianNoiseSampler(eye(2), 4, uint64(42));
draw = sampler.sampleNoise(2);
assert(isequal(size(draw), [2, 8]));
other = gtsam.GaussianNoiseSampler(eye(2), 4, uint64(42));
assert(isequal(draw, other.sampleControls(zeros(2, 2))));
rule = gtsam.MPPI(1.0);
assert(norm(rule.weights([0; log(2)]) - [2/3; 1/3]) < 1e-12);

params = gtsam.VanillaMPPIParams();
params.numSamples = 16;
params.controlCovariance = eye(2);
params.seed = uint64(24);
x0 = gtsam.symbol('x', 0);
x1 = gtsam.symbol('x', 1);
u0 = gtsam.symbol('u', 0);
keys = gtsam.KeyVector();
keys.push_back(u0);
states = gtsam.KeyVector();
states.push_back(x0);
states.push_back(x1);
for isPointMass = [true, false]
    graph = gtsam.NonlinearFactorGraph();
    initial = gtsam.Values();
    initial.insertPoint2(u0, zeros(2, 1));
    if isPointMass
        model = gtsam.PointMass(0.1);
        start = [0; 0; 1; 0];
        initial.insertVector4(x0, start);
        initial.insertVector4(x1, model.predict(start, zeros(2, 1)));
        graph.add(gtsam.PointMassDynamicsFactor(x0, u0, x1, model, ...
            gtsam.noiseModel.Constrained.All(4)));
        controller = gtsam.VanillaMPPIControllerPointMassDynamicsFactor(params);
        planner = gtsam.SamplingPlannerVector4Point2(states, keys, 0.1);
        dynamics = gtsam.pointMassVectorField();
    else
        model = gtsam.Unicycle(0.1);
        start = gtsam.Pose2();
        initial.insert(x0, start);
        initial.insert(x1, start);
        graph.add(gtsam.UnicycleDynamicsFactor(x0, u0, x1, model, ...
            gtsam.noiseModel.Constrained.All(3)));
        controller = gtsam.VanillaMPPIControllerUnicycleDynamicsFactor(params);
        planner = gtsam.SamplingPlannerPose2Point2(states, keys, 0.1);
        dynamics = gtsam.unicycleTransition(model);
    end
    graph.add(gtsam.PriorFactorPoint2(u0, [1; 0], gtsam.noiseModel.Unit.Create(2)));
    result = controller.optimize(graph, initial, keys);
    proposal = gtsam.GaussianProposal(eye(2), params.numSamples, params.seed);
    plan = planner.optimize(graph, initial, dynamics, proposal, rule, params.iterations);
    assert(norm(plan.atPoint2(u0) - result.atPoint2(u0)) < 1e-12);
    assert(abs(graph.error(plan) - graph.error(result)) < 1e-12);
    predicted = model.predict(start, result.atPoint2(u0));
    if isPointMass
        assert(norm(result.atVector4(x1) - predicted) < 1e-12);
    else
        assert(result.atPose2(x1).equals(predicted, 1e-12));
    end
    assert(graph.error(result) < graph.error(initial));
end
end
