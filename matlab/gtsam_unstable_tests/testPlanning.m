% Exercise the planning models and vector and pose trajectories.
% Authors: Akash Ratheesh and Frank Dellaert. See LICENSE for details.

function testPlanning()
sampler = gtsam.GaussianNoiseSampler(eye(2), 4, uint64(42));
draw = sampler.sampleNoise(2);
assert(isequal(size(draw), [2, 8]));
other = gtsam.GaussianNoiseSampler(eye(2), 4, uint64(42));
assert(isequal(draw, other.sampleControls(zeros(2, 2))));
rule = gtsam.MPPI(1.0);
assert(norm(rule.weights([0; log(2)]) - [2/3; 1/3]) < 1e-12);

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
    if isPointMass
        model = gtsam.PointMass(0.1);
        start = [0; 0; 1; 0];
        graph.add(gtsam.PointMassDynamicsFactor(x0, u0, x1, model, ...
            gtsam.noiseModel.Constrained.All(4)));
        planner = gtsam.SamplingPlannerVectorVector(states, keys, 0.1);
        dynamics = gtsam.pointMassVectorField();
    else
        model = gtsam.Unicycle(0.1);
        start = gtsam.Pose2();
        graph.add(gtsam.UnicycleDynamicsFactor(x0, u0, x1, model, ...
            gtsam.noiseModel.Constrained.All(3)));
        planner = gtsam.SamplingPlannerPose2Vector(states, keys, 0.1);
        dynamics = gtsam.unicycleTransition(model);
    end
    graph.add(gtsam.PriorFactorPoint2(u0, [1; 0], gtsam.noiseModel.Unit.Create(2)));
    controls = [0.2; -0.1];
    seed = gtsam.Values();
    seed.insert(x0, start);
    initial = planner.rollout(seed, controls, dynamics);
    assert(isequal(initial.atVector(u0), controls));
    predicted = model.predict(start, controls);
    if isPointMass
        assert(norm(initial.atVector(x1) - predicted) < 1e-12);
    else
        assert(initial.atPose2(x1).equals(predicted, 1e-12));
    end
    for searchRule = {rule, gtsam.CEM(4), gtsam.PredictiveSampling()}
        proposal = gtsam.GaussianProposal(eye(2), 16, uint64(24));
        result = planner.optimize(graph, initial, dynamics, proposal, searchRule{1}, 1);
        predicted = model.predict(start, result.atVector(u0));
        if isPointMass
            assert(norm(result.atVector(x1) - predicted) < 1e-12);
        else
            assert(result.atPose2(x1).equals(predicted, 1e-12));
        end
        assert(graph.error(result) < graph.error(initial));
    end
end
end
