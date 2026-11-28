function testPlanningLearning
% Exercise the unstable GPC and PO-MPC bindings on a short control horizon.
% Authors: Akash Ratheesh and Frank Dellaert.

flow = gtsam.GenerativePredictiveControl(2, 1, 3, 8, 42);
previous = [0.1, -0.2, 0.3];
generated = flow.generate([0; 1], previous, 1, 3);
assert(norm(generated - previous) < 1e-12);
proposal = gtsam.GPCProposal(flow, [0; 1], eye(1), 2, 2, 1, 3, 42);
search = proposal.initialize(previous);
batch = proposal.sample(search, gtsam.Values(), 1, true);
assert(isequal(size(batch.controls), [1 12]));
assert(norm(batch.controls(:, 7:12) - repmat(previous, 1, 2)) < 1e-12);
costs = [20, 0, 10; 3, 9, 0];
tail = gtsam.GenerativePredictiveControl.riskCosts(costs, 0.5);
assert(norm(tail - [25; 10.5] / 1.5) < 1e-12);

params = gtsam.PolicyOptimizationMPCParams();
params.horizon = 3;
params.samples = 16;
params.policySamples = 4;
params.elites = 4;
params.updateInterval = 0;
learner = gtsam.PolicyOptimizationMPC(2, 1, params);
state = [0; 1];
plan = learner.plan(state);
assert(isequal(size(plan.mean), [1 3]));
assert(all(plan.stddev >= params.minStddev));
search = learner.searchFrom(state, plan.mean, gtsam.GaussianNoiseSampler(0.09, 128, 42), ...
    32, gtsam.EliteMPPI(32, 1, 0.01), 1);
assert(isequal(size(search.mean), [1 3]));
learner.observe(state, plan, 0, state, true);
assert(learner.replaySize() == 1);
end
