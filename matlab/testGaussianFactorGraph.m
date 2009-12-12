%-----------------------------------------------------------------------
% equals
fg = createGaussianFactorGraph();
fg2 = createGaussianFactorGraph();
CHECK('equals',fg.equals(fg2,1e-9));

%-----------------------------------------------------------------------
% error
zero = createZeroDelta();
actual = fg.error(zero);
DOUBLES_EQUAL( 5.625, actual, 1e-9 );

%-----------------------------------------------------------------------
% eliminate_x1
fg = createGaussianFactorGraph();
actual = fg.eliminateOne('x1');

%-----------------------------------------------------------------------
% eliminate_x2
fg = createGaussianFactorGraph();
actual = fg.eliminateOne('x2');

%-----------------------------------------------------------------------
% eliminateAll
sigma1=[.1;.1];
I = eye(2);
R1 = I;
d1=[-.1;-.1];
cg1 = GaussianConditional('x1',d1, R1,sigma1);

sigma2=[0.149071; 0.149071];
R2 = I;
A1= -I;
d2=[0; .2];
cg2 = GaussianConditional('l1',d2, R2, 'x1', A1,sigma2);

sigma3=[0.0894427; 0.0894427];
R3 = I;
A21 = -0.2*I;
A22 = -0.8*I;
d3 =[.2; -.14];
cg3 = GaussianConditional('x2',d3, R3, 'l1', A21, 'x1', A22, sigma3);

expected = GaussianBayesNet;
expected.push_back(cg3);
expected.push_back(cg2);
expected.push_back(cg1);

% Check one ordering
fg1 = createGaussianFactorGraph();
ord1 = Ordering;
ord1.push_back('x2');
ord1.push_back('l1');
ord1.push_back('x1');
actual1 = fg1.eliminate_(ord1);
CHECK('eliminateAll', actual1.equals(expected,1e-5));

%-----------------------------------------------------------------------
% matrix

fg = createGaussianFactorGraph();
ord = Ordering;
ord.push_back('x1');
ord.push_back('x2');
ord.push_back('l1');

[H,z] = fg.matrix(ord);

