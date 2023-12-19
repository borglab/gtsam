Modeling Robot Motion
-----------------------

Modeling with Factor Graphs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before diving into a SLAM example, let us consider the simpler problem
of modeling robot motion. This can be done with a *continuous* Markov
chain, and provides a gentle introduction to GTSAM.

|image: 4\_Users\_dellaert\_git\_github\_doc\_images\_FactorGraph.png|
Figure 3: Factor graph for robot localization.

The factor graph for a simple example is shown in Figure
`3 <#fig_OdometryFG>`__. There are three variables :math:`x_{1}`,
:math:`x_{2}`, and :math:`x_{3}` which represent the poses of the robot
over time, rendered in the figure by the open-circle variable nodes. In
this example, we have one **unary factor**
:math:`f_{0}\left( x_{1} \right)` on the first pose :math:`x_{1}` that
encodes our prior knowledge about :math:`x_{1}`, and two **binary
factors** that relate successive poses, respectively
:math:`f_{1}\left( {x_{1},x_{2};o_{1}} \right)` and
:math:`f_{2}\left( {x_{2},x_{3};o_{2}} \right)`, where :math:`o_{1}` and
:math:`o_{2}` represent odometry measurements.

Creating a Factor Graph
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following C++ code, included in GTSAM as an example, creates the
factor graph in Figure `3 <#fig_OdometryFG>`__:

::

    // Create an empty nonlinear factor graph
    NonlinearFactorGraph graph;

    // Add a Gaussian prior on pose x_1
    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // Add two odometry factors
    Pose2 odometry(2.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr odometryNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

Above, line 2 creates an empty factor graph. We then add the factor
:math:`f_{0}\left( x_{1} \right)` on lines 5-8 as an instance of
***PriorFactor<T>***, a templated class provided in the slam subfolder,
with ***T=Pose2***. Its constructor takes a variable ***Key*** (in this
case 1), a mean of type ***Pose2,*** created on Line 5, and a noise
model for the prior density. We provide a diagonal Gaussian of type
***noiseModel::Diagonal*** by specifying three standard deviations in
line 7, respectively 30 cm. on the robot's position, and 0.1 radians on
the robot's orientation. Note that the ***Sigmas*** constructor returns
a shared pointer, anticipating that typically the same noise models are
used for many different factors.

Similarly, odometry measurements are specified as ***Pose2*** on line
11, with a slightly different noise model defined on line 12-13. We then
add the two factors :math:`f_{1}\left( {x_{1},x_{2};o_{1}} \right)` and
:math:`f_{2}\left( {x_{2},x_{3};o_{2}} \right)` on lines 14-15, as
instances of yet another templated class, ***BetweenFactor<T>***, again
with ***T=Pose2***.

When running the example (*make OdometryExample.run* on the command
prompt), it will print out the factor graph as follows:

::

    Factor Graph:
    size: 3
    Factor 0: PriorFactor on 1
      prior mean: (0, 0, 0)
      noise model: diagonal sigmas [0.3; 0.3; 0.1];
    Factor 1: BetweenFactor(1,2)
      measured: (2, 0, 0)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];
    Factor 2: BetweenFactor(2,3)
      measured: (2, 0, 0)
      noise model: diagonal sigmas [0.2; 0.2; 0.1];

Factor Graphs versus Values
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At this point it is instructive to emphasize two important design ideas
underlying GTSAM:

#. The factor graph and its embodiment in code specify the joint
   probability distribution :math:`P\left( X \middle| Z \right)` over
   the *entire* trajectory
   :math:`X = \left\{ {x_{1},x_{2},x_{3}} \right\}` of the robot, rather
   than just the last pose. This *smoothing* view of the world gives
   GTSAM its name: “smoothing and mapping”. Later in this document we
   will talk about how we can also use GTSAM to do filtering (which you
   often do *not* want to do) or incremental inference (which we do all
   the time).
#. A factor graph in GTSAM is just the specification of the probability
   density :math:`P\left( X \middle| Z \right)`, and the corresponding
   ***FactorGraph*** class and its derived classes do not ever contain a
   “solution”. Rather, there is a separate type ***Values*** that is
   used to specify specific values for (in this case) :math:`x_{1}`,
   :math:`x_{2}`, and :math:`x_{3}`, which can then be used to evaluate
   the probability (or, more commonly, the error) associated with
   particular values.

The latter point is often a point of confusion with beginning users of
GTSAM. It helps to remember that when designing GTSAM we took a
functional approach of classes corresponding to mathematical objects,
which are usually immutable. You should think of a factor graph as a
*function* to be applied to values -as the notation
:math:`f\left( X \right) \propto P\left( X \middle| Z \right)` implies-
rather than as an object to be modified.

Non-linear Optimization in GTSAM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The listing below creates a ***Values*** instance, and uses it as the
initial estimate to find the maximum a-posteriori (MAP) assignment for
the trajectory :math:`X`:

::

    // create (deliberately inaccurate) initial estimate
    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));

    // optimize using Levenberg-Marquardt optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

Lines 2-5 in Listing `2.4 <#listing_OdometryOptimize>`__ create the
initial estimate, and on line 8 we create a non-linear
Levenberg-Marquardt style optimizer, and call ***optimize*** using
default parameter settings. The reason why GTSAM needs to perform
non-linear optimization is because the odometry factors
:math:`f_{1}\left( {x_{1},x_{2};o_{1}} \right)` and
:math:`f_{2}\left( {x_{2},x_{3};o_{2}} \right)` are non-linear, as they
involve the orientation of the robot. This also explains why the factor
graph we created in Listing `2.2 <#listing_OdometryExample>`__ is of
type ***NonlinearFactorGraph***. The optimization class linearizes this
graph, possibly multiple times, to minimize the non-linear squared error
specified by the factors.

The relevant output from running the example is as follows:

::

    Initial Estimate:
    Values with 3 values:
    Value 1: (0.5, 0, 0.2)
    Value 2: (2.3, 0.1, -0.2)
    Value 3: (4.1, 0.1, 0.1)

    Final Result:
    Values with 3 values:
    Value 1: (-1.8e-16, 8.7e-18, -9.1e-19)
    Value 2: (2, 7.4e-18, -2.5e-18)
    Value 3: (4, -1.8e-18, -3.1e-18)

It can be seen that, subject to very small tolerance, the ground truth
solution :math:`x_{1} = \left( {0,0,0} \right)`,
:math:`x_{2} = \left( {2,0,0} \right)`, and
:math:`x_{3} = \left( {4,0,0} \right)` is recovered.

Full Posterior Inference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

GTSAM can also be used to calculate the covariance matrix for each pose
after incorporating the information from all measurements :math:`Z`.
Recognizing that the factor graph encodes the **posterior density**
:math:`P\left( X \middle| Z \right)`, the mean :math:`\mu` together with
the covariance :math:`\Sigma` for each pose :math:`x` approximate the
**marginal posterior density** :math:`P\left( x \middle| Z \right)`.
Note that this is just an approximation, as even in this simple case the
odometry factors are actually non-linear in their arguments, and GTSAM
only computes a Gaussian approximation to the true underlying posterior.

The following C++ code will recover the posterior marginals:

::

    // Query the marginals
    cout.precision(2);
    Marginals marginals(graph, result);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

The relevant output from running the example is as follows:

::

    x1 covariance:
           0.09     1.1e-47     5.7e-33
        1.1e-47        0.09     1.9e-17
        5.7e-33     1.9e-17        0.01
    x2 covariance:
           0.13     4.7e-18     2.4e-18
        4.7e-18        0.17        0.02
        2.4e-18        0.02        0.02
    x3 covariance:
           0.17     2.7e-17     8.4e-18
        2.7e-17        0.37        0.06
        8.4e-18        0.06        0.03

What we see is that the marginal covariance
:math:`P\left( x_{1} \middle| Z \right)` on :math:`x_{1}` is simply the
prior knowledge on :math:`x_{1}`, but as the robot moves the uncertainty
in all dimensions grows without bound, and the :math:`y` and
:math:`\theta` components of the pose become (positively) correlated.

An important fact to note when interpreting these numbers is that
covariance matrices are given in *relative* coordinates, not absolute
coordinates. This is because internally GTSAM optimizes for a change
with respect to a linearization point, as do all nonlinear optimization
libraries.
