% Exercise default and explicit active-set parameters through every LP overload.
function testLpProblem
import gtsam.*
key = symbol('x', uint64(2^40 + 42));
model = noiseModel.Unit.Create(1);
problem = LpProblem();
problem.addCost(JacobianFactor(key, -1, 0, model));
problem.addConstraint(LinearConstraint.LessEqual(JacobianFactor(key, 1, 1, model)));
problem.addConstraint(LinearConstraint.GreaterEqual(JacobianFactor(key, 1, 0, model)));
initial = Values(); initial.insert(key, 0);
params = ActiveSetSolverParams(); params.maxIterations = 50;
solvers = {ActiveSetSolver(problem), ActiveSetSolver(problem, params)};
results = {problem.optimize(), problem.optimize(initial), ...
           problem.optimize(params), problem.optimize(initial, params)};
for i = 1:numel(solvers)
    results{end+1} = solvers{i}.optimize();
    results{end+1} = solvers{i}.optimize(initial);
end
for i = 1:numel(results)
    CHECK('LP optimum', abs(results{i}.atVector(key) - 1) < 1e-7);
    CHECK('LP objective', abs(problem.objective(results{i}) + 1) < 1e-7);
end
end
