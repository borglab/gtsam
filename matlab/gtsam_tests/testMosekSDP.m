% Exercise both MOSEK formulations with and without homogeneous sharing.
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
           MosekMonolithicSDP(problem, false), ...
           MosekChordalSDP(problem, ChordalOrderingType.Colamd), ...
           MosekChordalSDP(problem, ChordalOrderingType.Colamd, false), ...
           MosekChordalSDP(problem, ChordalOrderingType.Metis), ...
           MosekChordalSDP(problem, ChordalOrderingType.Metis, false)};
for i = 1:numel(solvers)
    solver = solvers{i};
    if mod(i, 2)
        solved = solver.solve();
    else
        params = std.mapstringdouble();
        params.emplace('intpntCoTolRelGap', 1e-8);
        CHECK('Parameter map', params.size() == 1 && ...
              params.at('intpntCoTolRelGap') == 1e-8);
        solved = solver.solve(params);
    end
    CHECK('MOSEK solve', solved);
    CHECK('Nonzero analytic optimum', abs(solver.objectiveValue() - expected) < 1e-6);
    CHECK('Feasible status', contains(solver.problemStatus(), 'PrimalAndDualFeasible'));
    values = solver.qcqpValues();
    CHECK('Recovered anchor', norm(values.atMatrix(keys(1)) - [1; 1; 0]) < 1e-6);
    CHECK('Repeated retrieval', values.equals(solver.qcqpValues(), 1e-9));
    CHECK('Ordered keys', solver.orderedKeys().size() == 4);
    dimensions = solver.orderedKeyDims();
    evrs = solver.variableEVRs();
    CHECK('Diagnostic sizes', dimensions.size() == 4 && evrs.size() == 4);
    for j = 1:4
        CHECK('Dimension at symbolic key', dimensions.at(keys(j)) == 3);
        CHECK('Eigenvalue ratio', evrs.at(j - 1) > 1e4);
    end
    CHECK('Solver timing', solver.solveTimeSeconds() >= 0);
    if i > 2
        CHECK('Chordal Bayes tree', solver.bayesTree().size() > 0);
    end
end
CHECK('Empty parameter map', solver.solve(std.mapstringdouble()));
end
