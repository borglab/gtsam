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
    if mod(i, 2)
        solved = solver.solve();
    else
        solved = solver.solve(containers.Map({'intpntCoTolRelGap'}, {1e-8}));
    end
    CHECK('MOSEK solve', solved);
    CHECK('Nonzero analytic optimum', abs(solver.objectiveValue() - expected) < 1e-6);
    CHECK('Feasible status', contains(solver.problemStatus(), 'PrimalAndDualFeasible'));
    values = solver.qcqpValues();
    CHECK('Recovered anchor', norm(values.atMatrix(0) - [1; 1; 0]) < 1e-6);
    CHECK('Repeated retrieval', values.equals(solver.qcqpValues(), 1e-9));
    CHECK('Ordered keys', solver.orderedKeys().size() == 4);
    dimensions = solver.orderedKeyDims();
    CHECK('Original QCQP dimensions', dimensions.Count == 4 && ...
          all(cell2mat(dimensions.values()) == 3));
    evrs = solver.variableEVRs();
    CHECK('Eigenvalue ratios', isequal(size(evrs), [4, 1]) && all(evrs > 1e4));
    CHECK('Solver timing', solver.solveTimeSeconds() >= 0);
    if i > 2
        CHECK('Chordal Bayes tree', solver.bayesTree().size() > 0);
    end
end
CHECK('Empty parameter map', solver.solve(containers.Map('KeyType', 'char', ...
                                                       'ValueType', 'double')));

% Empty maps and symbolic keys must also survive native container conversion.
emptyValues = Values();
CHECK('Empty dimension map', emptyValues.dims().Count == 0);
key = symbol('x', 7);
emptyValues.insert(key, Pose2());
dimensions = emptyValues.dims();
CHECK('Exact symbolic key', strcmp(dimensions.KeyType, 'uint64') && ...
      dimensions(key) == 3);
end
