% Exercise natural D=1 coordinates and affine constraints without MOSEK.
function testQcqpProblem
import gtsam.*

rotation = Rot3.RzRyRx(0.1, -0.2, 0.3);
typedValues = {Rot2.fromAngle(0.3), rotation, ...
               Pose2(1, -2, 0.3), Pose3(rotation, [1; -2; 3])};
toQcqp = {@qcqpValueRot2, @qcqpValueRot3, @qcqpValuePose2, @qcqpValuePose3};
insert = {@insertQcqpValueRot2, @insertQcqpValueRot3, ...
          @insertQcqpValuePose2, @insertQcqpValuePose3};
fromQcqp = {@fromQcqpValueRot2, @fromQcqpValueRot3, ...
            @fromQcqpValuePose2, @fromQcqpValuePose3};
extract = {@extractQcqpValuesRot2, @extractQcqpValuesRot3, ...
           @extractQcqpValuesPose2, @extractQcqpValuesPose3};
dimensions = [2, 9, 6, 12];
expected = {[cos(0.3); sin(0.3)], reshape(rotation.matrix(), 9, 1), ...
            [cos(0.3); sin(0.3); -sin(0.3); cos(0.3); 1; -2], ...
            [reshape(rotation.matrix(), 9, 1); 1; -2; 3]};
keys = zeros(1, 4, 'uint64');
values = Values();
for i = 1:4
    keys(i) = symbol('x', i);
    vector = toQcqp{i}(typedValues{i});
    CHECK('Natural coordinate dimensions', isequal(size(vector), [dimensions(i), 1]));
    CHECK('Natural coordinate ordering', norm(vector - expected{i}) < 1e-12);
    recovered = fromQcqp{i}(vector);
    CHECK('D=1 conversion round trip', recovered.equals(typedValues{i}, 1e-12));
    insert{i}(keys(i), typedValues{i}, values);
    CHECK('Inserted QCQP value', norm(values.atMatrix(keys(i)) - vector) < 1e-12);
end

% Extract from a mixed Values object, preserving full-width symbolic keys.
rot2Values = extract{1}(values);
rot3Values = extract{2}(values);
pose2Values = extract{3}(values);
pose3Values = extract{4}(values);
CHECK('Extract Rot2', rot2Values.atRot2(keys(1)).equals(typedValues{1}, 1e-12));
CHECK('Extract Rot3', rot3Values.atRot3(keys(2)).equals(typedValues{2}, 1e-12));
CHECK('Extract Pose2', pose2Values.atPose2(keys(3)).equals(typedValues{3}, 1e-12));
CHECK('Extract Pose3', pose3Values.atPose3(keys(4)).equals(typedValues{4}, 1e-12));

% Hard Frobenius priors are exposed as fixed QCQP values even without MOSEK.
fixedKey = symbol('a', 0);
fixedGraph = NonlinearFactorGraph();
fixedGraph.add(FrobeniusPriorRot2(fixedKey, eye(2), noiseModel.Constrained.All(4)));
fixedProblem = QcqpProblem(fixedGraph);
fixedValues = fixedProblem.fixedVariables();
CHECK('Fixed anchor count', fixedValues.size() == 1);
CHECK('Fixed anchor value', norm(fixedValues.atMatrix(fixedKey) - [1; 0]) < 1e-12);

% Check the new overloads and the x'Ax + 2a'x convention through optimization.
key = symbol('p', 0);
A = 1;
a = 1;
constraint = QuadraticConstraint.Equal(key, A, a, 3, 1);
CHECK('Linear term present', constraint.hasLinearTerm());
CHECK('Linear term accessor', norm(constraint.a() - a) < 1e-12);
homogeneous = QuadraticConstraint.Equal(key, A, 3);
CHECK('Pure quadratic overload', ~homogeneous.hasLinearTerm());
scaled = QuadraticConstraint.Equal(key, A, 3, 2);
CHECK('Existing scalar sigma overload', ~scaled.hasLinearTerm() && ...
      scaled.b() == 3 && scaled.sigma() == 2);
vectorConstraint = QuadraticConstraint.Equal(key, eye(2), [1; -2], 3, 2);
CHECK('Vector linear term overload', vectorConstraint.hasLinearTerm() && ...
      norm(vectorConstraint.a() - [1; -2]) < 1e-12 && vectorConstraint.sigma() == 2);
explicit = QuadraticConstraint(key, A, a, 3, QuadraticConstraint.Sense.Equal, 1);
converted = QuadraticConstraint.FromPqr(key, 2, 2, -3, QuadraticConstraint.Sense.Equal);
for candidate = {constraint, explicit, converted}
    checkAffineConstraint(candidate{1}, key);
end

% FromPqr supports both the default sigma and an explicit sigma.
P = [4, 1; 1, 2];
q = [1; -2];
for sigma = [1, 2]
    if sigma == 1
        converted = QuadraticConstraint.FromPqr( ...
            key, P, q, -3, QuadraticConstraint.Sense.Equal);
    else
        converted = QuadraticConstraint.FromPqr( ...
            key, P, q, -3, QuadraticConstraint.Sense.Equal, sigma);
    end
    CHECK('FromPqr quadratic term', norm(converted.A() - P / 2) < 1e-12);
    CHECK('FromPqr linear term', norm(converted.a() - q / 2) < 1e-12);
    CHECK('FromPqr constant and sigma', converted.b() == 3 && converted.sigma() == sigma);
end
end

function checkAffineConstraint(constraint, key)
import gtsam.*
% Minimize 0.5*(x-2)^2 subject to x^2 + 2*x = 3. The nearest root is 1.
problem = QcqpProblem();
problem.addCost(QpCost(HessianFactor(key, 1, 2, 4)));
problem.addConstraint(constraint);
initial = Values();
initial.insert(key, 1.1);
params = AugmentedLagrangianParams();
params.updatePolicy = AugmentedLagrangianUpdatePolicy.Aggressive;
params.maxIterations = 100;
params.absoluteViolationTolerance = 1e-8;
params.relativeViolationTolerance = 1e-8;
params.absoluteCostTolerance = 1e-10;
params.relativeCostTolerance = 1e-10;
result = AugmentedLagrangianOptimizer(problem, initial, params).optimize();
CHECK('Affine constraint optimum', abs(result.atVector(key) - 1) < 1e-6);
end
