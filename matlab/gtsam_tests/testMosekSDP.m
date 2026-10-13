% Exercise natural-coordinate QCQPs with both MOSEK formulations.
function testMosekSDP
import gtsam.*
if exist('gtsam.MosekMonolithicSDP', 'class') ~= 8
    fprintf('Skipping MOSEK tests: toolbox built without MOSEK.\n');
    return
end

graph = NonlinearFactorGraph();
keys = zeros(1, 4, 'uint64');
for key = 0:3
    keys(key + 1) = symbol('x', key);
end
graph.add(FrobeniusPriorRot2(keys(1), eye(2), noiseModel.Constrained.All(4)));
for j = 1:4
    graph.add(FrobeniusBetweenFactorRot2(keys(j), keys(mod(j, 4) + 1), ...
                                       Rot2.fromAngle(0.2)));
end
problem = QcqpProblem(graph);
expected = 8 * (1 - cos(0.2));
solvers = {MosekMonolithicSDP(problem), ...
           MosekChordalSDP(problem, ChordalOrderingType.Colamd), ...
           MosekChordalSDP(problem, ChordalOrderingType.Metis)};
params = std.mapstringdouble();
params.emplace('intpntCoTolRelGap', 1e-8);
CHECK('Parameter map', params.size() == 1 && ...
      params.at('intpntCoTolRelGap') == 1e-8);
for i = 1:numel(solvers)
    solver = solvers{i};
    CHECK('MOSEK default solve', solver.solve());
    CHECK('MOSEK parameterized solve', solver.solve(params));
    CHECK('Nonzero analytic optimum', abs(solver.objectiveValue() - expected) < 1e-6);
    CHECK('Feasible status', contains(solver.problemStatus(), 'PrimalAndDualFeasible'));
    values = solver.qcqpValues();
    anchor = values.atMatrix(keys(1));
    CHECK('Natural anchor size', isequal(size(anchor), [2, 1]));
    CHECK('Recovered anchor', norm(anchor - [1; 0]) < 1e-6);
    CHECK('Repeated retrieval', values.equals(solver.qcqpValues(), 1e-9));
    CHECK('Ordered keys', solver.orderedKeys().size() == 4);
    dimensions = solver.orderedKeyDims();
    evrs = solver.variableEVRs();
    CHECK('Diagnostic sizes', dimensions.size() == 4 && evrs.size() == 4);
    for j = 1:4
        CHECK('Dimension at symbolic key', dimensions.at(keys(j)) == 2);
        CHECK('Eigenvalue ratio', evrs.at(j - 1) > 1e4);
    end
    CHECK('Solver timing', solver.solveTimeSeconds() >= 0);
    if isa(solver, 'gtsam.MosekChordalSDP')
        CHECK('Chordal Bayes tree', solver.bayesTree().size() > 0);
    end
    CHECK('Empty parameter map', solver.solve(std.mapstringdouble()));
end

% Linear and constant costs also work for keys without manifold constraints.
key = symbol('p', 0);
problem = QcqpProblem();
problem.addCost(QpCost(HessianFactor(key, 2 * eye(2), [1; 1], 2)));
checkPointProblem(problem, key, [0.5; 0.5], 0.5);

% The shifted-circle projection exercises the 2a convention in the SDP lift.
target = [4; 5];
center = [1; 1];
problem = QcqpProblem();
problem.addCost(QpCost(HessianFactor(key, eye(2), target, target' * target)));
problem.addConstraint(QuadraticConstraint.Equal(key, eye(2), -center, -1, 1));
projection = center + (target - center) / norm(target - center);
checkPointProblem(problem, key, projection, 0.5 * (norm(target - center) - 1)^2);
end

function checkPointProblem(problem, key, expectedValue, expectedCost)
import gtsam.*
solvers = {MosekMonolithicSDP(problem), ...
           MosekChordalSDP(problem, ChordalOrderingType.Colamd), ...
           MosekChordalSDP(problem, ChordalOrderingType.Metis)};
for i = 1:numel(solvers)
    solver = solvers{i};
    CHECK('Point problem solve', solver.solve());
    CHECK('Point problem optimum', abs(solver.objectiveValue() - expectedCost) < 1e-6);
    values = solver.qcqpValues();
    CHECK('Point problem recovery', norm(values.atMatrix(key) - expectedValue) < 1e-5);
    dimensions = solver.orderedKeyDims();
    CHECK('Point problem dimension', dimensions.at(key) == 2);
end
end
