%-----------------------------------------------------------------------
% solve

expected = [15.0471 ; -18.8824];

% create a conditional gaussion node
A1 =[1 2; 3 4];
A2 = [6 0.2;8 0.4];
R = [0.1 0.3; 0.0 0.34];
d=[0.2;0.5];
tau=[1;.34];


cg = ConditionalGaussian('x',d, R, 'x1', A1, 'l1', A2, tau);

sx1 = [0.2;0.5];
sl1 = [0.5;0.8];

%solution = FGConfig;
solution.insert('x1', sx1);
solution.insert('l1', sl1);

result = cg.solve(solution);

if(~all( abs(expected - result) < 0.0001 )) warning('solve failed'); end
    
%-----------------------------------------------------------------------

