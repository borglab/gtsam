% Direct Gaussian ALM binding smoke tests. Run on a MATLAB-capable build.
function testAugmentedLagrangianSolver
import gtsam.*
key = symbol('x', uint64(2^40 + 17));
costs = GaussianFactorGraph();
costs.add(HessianFactor(key, eye(2), [2; -1], 0));
equalities = GaussianFactorGraph();
equalities.add(JacobianFactor(key, [1, 1], 1, noiseModel.Isotropic.Sigma(1, 2)));
inequalities = GaussianFactorGraph();
inequalities.add(JacobianFactor(key, [1, 0], .5, noiseModel.Isotropic.Sigma(1, 3)));
params = AugmentedLagrangianParams();
params.absoluteStationarityTolerance = 1e-8;
params.absoluteViolationTolerance = 1e-8;
params.storeOptProgress = true;
params.enableProfiling = true;
lm = params.lmParams;
lm.setMaxIterations(80);
ordering = Ordering(); ordering.push_back(key);
lm.setOrdering(ordering);
params.lmParams = lm;
CHECK('ALM LM parameters', params.lmParams.getMaxIterations() == 80);
multifrontal = lm.multifrontalParams;
multifrontal.qrMode = gtsam.MultifrontalParameters.QRMode.Off;
lm.multifrontalParams = multifrontal;
params.lmParams = lm;
CHECK('ALM QR option', params.lmParams.multifrontalParams.qrMode == ...
    gtsam.MultifrontalParameters.QRMode.Off);
solver = AugmentedLagrangianSolver(costs, equalities, inequalities, params);
initial = VectorValues(); initial.insert(key, [10; -20]);
result = solver.optimizeWithState(initial);
CHECK('ALM convergence', result.converged);
CHECK('ALM solution', norm(result.values.at(key) - [.5; .5]) < 1e-6);
CHECK('ALM equality multipliers', abs(result.lambdaEq.at(0) + 3) < 1e-5);
CHECK('ALM inequality multipliers', abs(result.lambdaIneq.at(0) - 9) < 1e-5);
history = solver.progress();
CHECK('ALM history', history.size() > 1 && history.at(history.size()-1).converged);
CHECK('ALM counters', result.statistics.modelLinearizations == 0 && ...
    result.statistics.objectiveFactorCopies == 1);
for i = 1:3
    result = solver.optimizeWithState();
    CHECK('ALM repeated retrieval', norm(result.values.at(key) - [.5; .5]) < 1e-6);
end
empty = GaussianFactorGraph();
plain = AugmentedLagrangianSolver(costs, empty, empty);
CHECK('ALM default constructor parameters', norm(plain.optimize().at(key) - [2; -1]) < 1e-6);
problem = QpProblem(); problem.addCost(HessianFactor(key, eye(4), [1; 3; -2; 4], 0));
values = Values(); values.insert(key, zeros(2, 2));
solved = problem.optimize(values, params);
CHECK('ALM convenience driver', norm(solved.atMatrix(key) - [1, -2; 3, 4]) < 1e-6);
CHECK('ALM cold QP options', norm(problem.optimize(params).atVector(key) - [1; 3; -2; 4]) < 1e-6);
lm = params.lmParams; lm.setMaxIterations(0); params.lmParams = lm;
CHECK('ALM QP option forwarding', norm(problem.optimize(params).atVector(key)) == 0);
end
