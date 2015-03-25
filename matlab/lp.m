import gtsam.*
g = [-1; -1]  % min -x1-x2
C = [-1 0
    0 -1
    1 2
    4 2
    -1 1]';
b =[0;0;4;12;1]

%% step 0
m = length(C);
active = zeros(1, m);
x0 = [0;0];

for iter = 1:2
    % project -g onto the nullspace of active constraints in C to obtain the moving direction
    % It boils down to solving the following constrained linear least squares
    % system:  min_d//  || d// - d ||^2
    %           s.t. C*d// = 0
    % where d = -g, the opposite direction of the objective's gradient vector
    dgraph = GaussianFactorGraph
    dgraph.push_back(JacobianFactor(1, eye(2), -g, noiseModel.Unit.Create(2)));
    for i=1:m
        if (active(i)==1)
            ci = C(:,i);
            dgraph.push_back(JacobianFactor(1, ci', 0, noiseModel.Constrained.All(1)));
        end
    end
    d = dgraph.optimize.at(1)
    
    % Find the bad active constraints and remove them
    % TODO: FIXME Is this implementation correct?
    for i=1:m
        if (active(i) == 1)
            ci = C(:,i);
            if ci'*d < 0
                active(i) = 0;
            end
        end
    end
    active
    
    % We are going to jump:
    %       x1 = x0 + k*d;, k>=0
    % So check all inactive constraints that block the jump to find the smallest k
    % ci*x1 - bi <=0
    % ci*(x0 + k*d) - bi <= 0
    % ci*x0-bi + ci*k*d <= 0
    % - if ci*d < 0: great, no prob (-ci and d have the same direction)
    % - if ci*d > 0: (-ci and d have opposite directions)
    %     k <= -(ci*x0 - bi)/(ci*d)
    k = 100000;
    newActive = -1;
    for i=1:m
        if active(i) == 1
            continue
        end
        ci = C(:,i);
        if ci'*d > 0
            foundNewActive = true;
            if k > -(ci'*x0 - b(i))/(ci'*d)
                k = -(ci'*x0 - b(i))/(ci'*d);
                newActive = i;
            end
        end
    end
    
    % If there is no blocking constraint, the problem is unbounded
    if newActive == -1
        disp('Unbounded')
    else
        % Otherwise, make the jump, and we have a new active constraint
        x0 = x0 + k*d
        active(newActive) = 1
    end
    
end