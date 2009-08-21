%-----------------------------------------------------------------------
% equals
fg = createLinearFactorGraph();
fg2 = createLinearFactorGraph();
CHECK('equals',fg.equals(fg2));

%-----------------------------------------------------------------------
% error
cfg = createZeroDelta();
actual = fg.error(cfg);
DOUBLES_EQUAL( 5.625, actual, 1e-9 );

%-----------------------------------------------------------------------
% combine_factors_x1
fg = createLinearFactorGraph();
actual = fg.combine_factors('x1');
Al1 = [
   0., 0.
   0., 0.
   0., 0.
   0., 0.
   5., 0.
   0., 5.
  ];
                     
Ax1 = [
  10.,   0.
  0.00, 10.
  -10.,  0.
  0.00,-10.
  -5.,   0.
  00.,  -5.
  ];

Ax2 = [
   0., 0.
   0., 0.
   10., 0.
   +0.,10.
   0., 0.
   0., 0.
  ];

b=[-1;-1;2;-1;0;1];

expected = LinearFactor('l1',Al1,'x1',Ax1,'x2',Ax2,b);
CHECK('combine_factors_x1', actual.equals(expected,1e-9));

%-----------------------------------------------------------------------
% combine_factors_x2
fg = createLinearFactorGraph();
actual = fg.combine_factors('x2');

%-----------------------------------------------------------------------
% eliminate_x1
fg = createLinearFactorGraph();
actual = fg.eliminate_one('x1');

%-----------------------------------------------------------------------
% eliminate_x2
fg = createLinearFactorGraph();
actual = fg.eliminate_one('x2');

%-----------------------------------------------------------------------
% eliminateAll

R1 = [10, 0.0
    0.0, 10];
d1=[-1;-1];
cg1 = ConditionalGaussian(d1, R1);

R2 = [6.7082, 0.0
    0.0, 6.7082];
A1= [ -6.7082, 0.0
    0.0, -6.7082];
d2=[0;1.34164];
cg2 = ConditionalGaussian(d2, R2, 'x1', A1);

R3 = [11.1803, 0.0
    0.0, 11.1803];
A21 = [ -2.23607, 0.0
    0.0, -2.23607];
A22 = [-8.94427, 0.0
    0.0, -8.94427];
d3 =[2.23607; -1.56525];
cg3 = ConditionalGaussian(d3, R3, 'l1', A21, 'x1', A22);

expected = ChordalBayesNet;
expected.insert('x1', cg1);
expected.insert('l1', cg2);
expected.insert('x2', cg3);

% Check one ordering
fg1 = createLinearFactorGraph();
ord1 = Ordering;
ord1.push_back('x2');
ord1.push_back('l1');
ord1.push_back('x1');
actual1 = fg1.eliminate(ord1);
CHECK('eliminateAll', actual1.equals(expected));

%-----------------------------------------------------------------------
% matrix

fg = createLinearFactorGraph();
ord = Ordering;
ord.push_back('x2');
ord.push_back('l1');
ord.push_back('x1');

A = fg.matrix(ord);

