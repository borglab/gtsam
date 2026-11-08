% Exercise the shared formulation option with both QP backends.
function testQpFormulations
import gtsam.*
key = symbol('x', 42);
problem = QpProblem();
problem.addCost(JacobianFactor(key, eye(2), [2; 3], noiseModel.Unit.Create(2)));
problem.addConstraint(LinearConstraint.LessEqual( ...
    JacobianFactor(key, [1, 0], 1, noiseModel.Unit.Create(1))));
params = ActiveSetSolverParams();
CHECK('Default Automatic', params.qpFormulation == ActiveSetSolverParams.QpFormulation.Automatic);
CHECK('Default Sparse', params.qpSubproblemSolver == ActiveSetSolverParams.QpSubproblemSolver.Sparse);
CHECK('Default equality projection', params.projectDenseQpEqualities);
initial = Values();
initial.insert(key, [20; -10]);
solver = ActiveSetSolver(problem, params);
values = solver.optimize(initial);
CHECK('Default infeasible start', norm(values.atVector(key) - [1; 3]) < 1e-7);
for backend = {ActiveSetSolverParams.QpSubproblemSolver.Dense, ...
               ActiveSetSolverParams.QpSubproblemSolver.Sparse}
    params.qpSubproblemSolver = backend{1};
    for mode = {ActiveSetSolverParams.QpFormulation.Primal, ...
                ActiveSetSolverParams.QpFormulation.Dual, ...
                ActiveSetSolverParams.QpFormulation.Automatic}
        params.qpFormulation = mode{1};
        solver = ActiveSetSolver(problem, params);
        values = solver.optimize();
        CHECK('Formulation optimum', norm(values.atVector(key) - [1; 3]) < 1e-7);
    end
    params.qpFormulation = ActiveSetSolverParams.QpFormulation.Dual;
    initial = Values();
    initial.insert(key, [20; -10]);
    solver = ActiveSetSolver(problem, params);
    values = solver.optimize(initial);
    CHECK('Infeasible dual start', norm(values.atVector(key) - [1; 3]) < 1e-7);
end
problem.addConstraint(LinearConstraint.Equal( ...
    JacobianFactor(key, [1, 1], 1, noiseModel.Unit.Create(1))));
params.qpSubproblemSolver = ActiveSetSolverParams.QpSubproblemSolver.Dense;
for project = [false, true]
    params.projectDenseQpEqualities = project;
    CHECK('Projection option round trip', params.projectDenseQpEqualities == project);
    for mode = {ActiveSetSolverParams.QpFormulation.Primal, ...
                ActiveSetSolverParams.QpFormulation.Dual}
        params.qpFormulation = mode{1};
        solver = ActiveSetSolver(problem, params);
        values = solver.optimize();
        CHECK('Equality optimum', norm(values.atVector(key) - [0; 1]) < 1e-7);
    end
end
end
