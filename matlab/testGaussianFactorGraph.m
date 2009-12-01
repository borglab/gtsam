%-----------------------------------------------------------------------
% equals
fg = createGaussianFactorGraph();
fg2 = createGaussianFactorGraph();
CHECK('equals',fg.equals(fg,fg2));

%-----------------------------------------------------------------------
% error
cfg = createZeroDelta();
actual = fg.error(cfg);
DOUBLES_EQUAL( 5.625, actual, 1e-9 );

%-----------------------------------------------------------------------
% combine_factors_x1
fg = createGaussianFactorGraph();
%actual = fg.combine_factors('x1');
actual = fg.combined('x1');
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

expected = GaussianFactor('l1',Al1,'x1',Ax1,'x2',Ax2,b);
CHECK('combine_factors_x1', actual.equals(expected,1e-9));

%-----------------------------------------------------------------------
% combine_factors_x2
fg = createGaussianFactorGraph();
actual = fg.combine_factors('x2');

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
sigma1=.1;
R1 = eye(2);
d1=[-.1;-.1];
cg1 = ConditionalGaussian('x1',d1, R1,sigma1);

sigma2=0.149071;
R2 = eye(2);
A1= -eye(2);
d2=[0; .2];
cg2 = ConditionalGaussian('l1',d2, R2, 'x1', A1,sigma2);

sigma3=0.0894427;
R3 = eye(2);
A21 = [ -.2, 0.0
    0.0, -.2];
A22 = [-.8, 0.0
    0.0, -.8];
d3 =[.2; -.14];
cg3 = ConditionalGaussian('x2',d3, R3, 'l1', A21, 'x1', A22, sigma3);

expected = GaussianBayesNet;
expected.push_back(cg1);
expected.push_back(cg2);
expected.push_back(cg3);
expected.print_();
% Check one ordering
fg1 = createGaussianFactorGraph();
ord1 = Ordering;
ord1.push_back('x2');
ord1.push_back('l1');
ord1.push_back('x1');
actual1 = fg1.eliminate_(ord1);
actual1.print();
%CHECK('eliminateAll', actual1.equals(expected));

%-----------------------------------------------------------------------
% matrix

fg = createGaussianFactorGraph();
ord = Ordering;
ord.push_back('x2');
ord.push_back('l1');
ord.push_back('x1');

A = fg.matrix(ord);

