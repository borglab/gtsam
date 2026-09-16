% Exercise both MOSEK formulations with and without homogeneous sharing.
function testMosekSDP
import gtsam.*
if exist('gtsam.MosekMonolithicSDP', 'class') ~= 8
    fprintf('Skipping MOSEK tests: toolbox built without MOSEK.\n');
    return
end

graph = NonlinearFactorGraph();
graph.add(FrobeniusPriorRot2(0, eye(2), noiseModel.Constrained.All(4)));
for key = 0:3
    graph.add(FrobeniusBetweenFactorRot2(key, mod(key + 1, 4), ...
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
    CHECK('MOSEK solve', solver.solve());
    CHECK('Nonzero analytic optimum', abs(solver.objectiveValue() - expected) < 1e-6);
    CHECK('Optimal status', strcmpi(solver.problemStatus(), 'PrimalAndDualFeasible'));
    values = solver.qcqpValues();
    CHECK('Recovered anchor', norm(values.atMatrix(0) - [1; 1; 0]) < 1e-6);
    CHECK('Repeated retrieval', values.equals(solver.qcqpValues(), 1e-9));
    CHECK('Ordered keys', solver.orderedKeys().size() == 4);
    CHECK('Solver timing', solver.solveTimeSeconds() >= 0);
end
end
