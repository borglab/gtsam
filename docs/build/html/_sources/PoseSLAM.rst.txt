PoseSLAM
----------

Loop Closure Constraints
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The simplest instantiation of a SLAM problem is **PoseSLAM**, which
avoids building an explicit map of the environment. The goal of SLAM is
to simultaneously localize a robot and map the environment given
incoming sensor measurements (`Durrant-Whyte and Bailey,
2006 <#LyXCite-DurrantWhyte06ram>`__). Besides wheel odometry, one of
the most popular sensors for robots moving on a plane is a 2D
laser-range finder, which provides both odometry constraints between
successive poses, and loop-closure constraints when the robot re-visits
a previously explored part of the environment.

|image: 8\_Users\_dellaert\_git\_github\_doc\_images\_FactorGraph3.png|
Figure 6: Factor graph for PoseSLAM.

A factor graph example for PoseSLAM is shown in Figure
`6 <#fig_Pose2SLAM>`__. The following C++ code, included in GTSAM as an
example, creates this factor graph in code:

::

    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

    // Add odometry factors
    noiseModel::Diagonal::shared_ptr model =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
    graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
    graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
    graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

    // Add the loop closure constraint
    graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

As before, lines 1-4 create a nonlinear factor graph and add the unary
factor :math:`f_{0}\left( x_{1} \right)`. As the robot travels through
the world, it creates binary factors
:math:`f_{t}\left( {x_{t},x_{t + 1}} \right)` corresponding to odometry,
added to the graph in lines 6-12 (Note that M\_PI\_2 refers to pi/2).
But line 15 models a different event: a **loop closure**. For example,
the robot might recognize the same location using vision or a laser
range finder, and calculate the geometric pose constraint to when it
first visited this location. This is illustrated for poses :math:`x_{5}`
and :math:`x_{2}`, and generates the (red) loop closing factor
:math:`f_{5}\left( {x_{5},x_{2}} \right)`.

|image: 9\_Users\_dellaert\_git\_github\_doc\_images\_example1.png|
Figure 7: The result of running optimize on the factor graph in Figure
`6 <#fig_Pose2SLAM>`__.

We can optimize this factor graph as before, by creating an initial
estimate of type ***Values***, and creating and running an optimizer.
The result is shown graphically in Figure `7 <#fig_example>`__, along
with covariance ellipses shown in green. These covariance ellipses in 2D
indicate the marginal over position, over all possible orientations, and
show the area which contain 68.26% of the probability mass (in 1D this
would correspond to one standard deviation). The graph shows in a clear
manner that the uncertainty on pose :math:`x_{5}` is now much less than
if there would be only odometry measurements. The pose with the highest
uncertainty, :math:`x_{4}`, is the one furthest away from the unary
constraint :math:`f_{0}\left( x_{1} \right)`, which is the only factor
tying the graph to a global coordinate frame.

The figure above was created using an interface that allows you to use
GTSAM from within MATLAB, which provides for visualization and rapid
development. We discuss this next.

Using the MATLAB Interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A large subset of the GTSAM functionality can be accessed through
wrapped classes from within MATLAB
(GTSAM also allows you to wrap your own custom-made classes, although
this is outside the scope of this manual).
The following code excerpt is the MATLAB equivalent of the C++ code in
Listing `4.1 <#listing_Pose2SLAMExample>`__:

::

    graph = NonlinearFactorGraph;
    priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
    graph.add(PriorFactorPose2(1, Pose2(0, 0, 0), priorNoise));

    %% Add odometry factors
    model = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
    graph.add(BetweenFactorPose2(1, 2, Pose2(2, 0, 0   ), model));
    graph.add(BetweenFactorPose2(2, 3, Pose2(2, 0, pi/2), model));
    graph.add(BetweenFactorPose2(3, 4, Pose2(2, 0, pi/2), model));
    graph.add(BetweenFactorPose2(4, 5, Pose2(2, 0, pi/2), model));

    %% Add pose constraint
    graph.add(BetweenFactorPose2(5, 2, Pose2(2, 0, pi/2), model));

Note that the code is almost identical, although there are a few syntax
and naming differences:

-  Objects are created by calling a constructor instead of allocating
   them on the heap.
-  Namespaces are done using dot notation, i.e.,
   ***noiseModel::Diagonal::SigmasClasses*** becomes
   ***noiseModel.Diagonal.Sigmas***.
-  ***Vector*** and ***Matrix*** classes in C++ are just
   vectors/matrices in MATLAB.
-  As templated classes do not exist in MATLAB, these have been
   hardcoded in the GTSAM interface, e.g., ***PriorFactorPose2***
   corresponds to the C++ class ***PriorFactor<Pose2>***, etc.

After executing the code, you can call *whos* on the MATLAB command
prompt to see the objects created. Note that the indicated *Class*
corresponds to the wrapped C++ classes:

::

    >> whos
      Name                 Size            Bytes  Class
      graph                1x1               112  gtsam.NonlinearFactorGraph
      priorNoise           1x1               112  gtsam.noiseModel.Diagonal
      model                1x1               112  gtsam.noiseModel.Diagonal
      initialEstimate      1x1               112  gtsam.Values
      optimizer            1x1               112  gtsam.LevenbergMarquardtOptimizer

In addition, any GTSAM object can be examined in detail, yielding
identical output to C++:

::

    >> priorNoise
    diagonal sigmas [0.3; 0.3; 0.1];

    >> graph
    size: 6
    factor 0: PriorFactor on 1
      prior mean: (0, 0, 0)
      noise model: diagonal sigmas [0.3; 0.3; 0.1];
    factor 1: BetweenFactor(1,2)
      measured: (2, 0, 0)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];
    factor 2: BetweenFactor(2,3)
      measured: (2, 0, 1.6)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];
    factor 3: BetweenFactor(3,4)
      measured: (2, 0, 1.6)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];
    factor 4: BetweenFactor(4,5)
      measured: (2, 0, 1.6)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];
    factor 5: BetweenFactor(5,2)
      measured: (2, 0, 1.6)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];

And it does not stop there: we can also call some of the functions
defined for factor graphs. E.g.,

::

    >> graph.error(initialEstimate)
    ans =
       20.1086

    >> graph.error(result)
    ans =
       8.2631e-18

computes the sum-squared error
:math:`\frac{1}{2}\sum\limits_{i}{||h_{i}\left( X_{i} \right) - z_{i}||}_{\Sigma}^{2}{}`
before and after optimization.

Reading and Optimizing Pose Graphs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|image: 10\_Users\_dellaert\_git\_github\_doc\_images\_w100-result.png|
Figure 8: MATLAB plot of small Manhattan world example with 100 poses
(due to Ed Olson). The initial estimate is shown in green. The optimized
trajectory, with covariance ellipses, in blue.

The ability to work in MATLAB adds a much quicker development cycle, and
effortless graphical output. The optimized trajectory in Figure
`8 <#fig_w100>`__ was produced by the code below, in which *load2D*
reads TORO files. To see how plotting is done, refer to the full source
code.
::

    %% Initialize graph, initial estimate, and odometry noise
    datafile = findExampleDataFile('w100.graph');
    model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 5*pi/180]);
    [graph,initial] = load2D(datafile, model);

    %% Add a Gaussian prior on pose x_0
    priorMean = Pose2(0, 0, 0);
    priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
    graph.add(PriorFactorPose2(0, priorMean, priorNoise));

    %% Optimize using Levenberg-Marquardt optimization and get marginals
    optimizer = LevenbergMarquardtOptimizer(graph, initial);
    result = optimizer.optimizeSafely;
    marginals = Marginals(graph, result);

PoseSLAM in 3D
~~~~~~~~~~~~~~~~~~

PoseSLAM can easily be extended to 3D poses, but some care is needed to
update 3D rotations. GTSAM supports both **quaternions** and
:math:`3 \times 3` **rotation matrices** to represent 3D rotations. The
selection is made via the compile flag GTSAM\_USE\_QUATERNIONS.

|image:
11\_Users\_dellaert\_git\_github\_doc\_images\_sphere2500-result.png|
Figure 9: 3D plot of sphere example (due to Michael Kaess). The very
wrong initial estimate, derived from odometry, is shown in green. The
optimized trajectory is shown red. Code below:

::

    %% Initialize graph, initial estimate, and odometry noise
    datafile = findExampleDataFile('sphere2500.txt');
    model = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
    [graph,initial] = load3D(datafile, model, true, 2500);
    plot3DTrajectory(initial, 'g-', false); % Plot Initial Estimate

    %% Read again, now with all constraints, and optimize
    graph = load3D(datafile, model, false, 2500);
    graph.add(NonlinearEqualityPose3(0, initial.atPose3(0)));
    optimizer = LevenbergMarquardtOptimizer(graph, initial);
    result = optimizer.optimizeSafely();
    plot3DTrajectory(result, 'r-', false); axis equal;
