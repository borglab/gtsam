Robot Localization
--------------------

Unary Measurement Factors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this section we add measurements to the factor graph that will help
us actually *localize* the robot over time. The example also serves as a
tutorial on creating new factor types.

|image: 5\_Users\_dellaert\_git\_github\_doc\_images\_FactorGraph2.png|
Figure 4: Robot localization factor graph with unary measurement factors
at each time step.

In particular, we use **unary measurement factors** to handle external
measurements. The example from Section `2 <#sec_Robot_Localization>`__
is not very useful on a real robot, because it only contains factors
corresponding to odometry measurements. These are imperfect and will
lead to quickly accumulating uncertainty on the last robot pose, at
least in the absence of any external measurements (see Section
`2.5 <#subsec_Full_Posterior_Inference>`__). Figure
`4 <#fig_LocalizationFG>`__ shows a new factor graph where the prior
:math:`f_{0}\left( x_{1} \right)` is omitted and instead we added three
unary factors :math:`f_{1}\left( {x_{1};z_{1}} \right)`,
:math:`f_{2}\left( {x_{2};z_{2}} \right)`, and
:math:`f_{3}\left( {x_{3};z_{3}} \right)`, one for each localization
measurement :math:`z_{t}`, respectively. Such unary factors are
applicable for measurements :math:`z_{t}` that depend *only* on the
current robot pose, e.g., GPS readings, correlation of a laser
range-finder in a pre-existing map, or indeed the presence of absence of
ceiling lights (see `Dellaert et al. <#LyXCite-Dellaert99b>`__ (1999)
for that amusing example).

Defining Custom Factors
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In GTSAM, you can create custom unary factors by deriving a new class
from the built-in class ***NoiseModelFactor1<T>***, which implements a
unary factor corresponding to a measurement likelihood with a Gaussian
noise model,
:math:`L\left( q;m \right)\operatorname{=\ exp}\left\{ - \frac{1}{2}{||h\left( q \right) - m||}_{\Sigma}^{2} \right\} = f\left( q \right)`
where :math:`m` is the measurement, :math:`q` is the unknown variable,
:math:`h\left( q \right)` is a (possibly nonlinear) measurement
function, and :math:`\Sigma` is the noise covariance. Note that
:math:`m` is considered *known* above, and the likelihood
:math:`L\left( {q;m} \right)` will only ever be evaluated as a function
of :math:`q`, which explains why it is a unary factor
:math:`f\left( q \right)`. It is always the unknown variable :math:`q`
that is either likely or unlikely, given the measurement.

**Note:** many people get this backwards, often misled by the
conditional density notation :math:`P\left( m \middle| q \right)`. In
fact, the likelihood :math:`L\left( {q;m} \right)` is *defined* as any
function of :math:`q` proportional to
:math:`P\left( m \middle| q \right)`.

Listing `3.2 <#listing_LocalizationFactor>`__ shows an example on how to
define the custom factor class ***UnaryFactor*** which implements a
“GPS-like” measurement likelihood:

::

    class UnaryFactor: public NoiseModelFactor1<Pose2> {
      double mx_, my_; ///< X and Y measurements

    public:
      UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
        NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

      Vector evaluateError(const Pose2& q,
                           boost::optional<Matrix&> H = boost::none) const
      {
        if (H) (*H) = (Matrix(2,3)<< 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
      }
    };

In defining the derived class on line 1, we provide the template
argument ***Pose2*** to indicate the type of the variable :math:`q`,
whereas the measurement is stored as the instance variables ***mx\_***
and ***my\_***, defined on line 2. The constructor on lines 5-6 simply
passes on the variable key :math:`j` and the noise model to the
superclass, and stores the measurement values provided. The most
important function to has be implemented by every factor class is
***evaluateError***, which should return
:math:`E\left( q \right) = {h\left( q \right) - m}` which is done on
line 12. Importantly, because we want to use this factor for nonlinear
optimization (see e.g., `Dellaert and Kaess
2006 <#LyXCite-Dellaert06ijrr>`__ for details), whenever the optional
argument :math:`H` is provided, a ***Matrix*** reference, the function
should assign the **Jacobian** of :math:`h\left( q \right)` to it,
evaluated at the provided value for :math:`q`. This is done for this
example on line 11. In this case, the Jacobian of the 2-dimensional
function :math:`h`, which just returns the position of the robot,

.. math::

   h\left( q \right) = \left\lbrack \begin{array}{l}
   q_{x} \\
   q_{y} \\
   \end{array} \right\rbrack

with respect the 3-dimensional pose
:math:`q = \left( {q_{x},q_{y},q_{\theta}} \right)`, yields the
following simple :math:`2 \times 3` matrix:

.. math::

   H = \left\lbrack \begin{array}{lll}
   1 & 0 & 0 \\
   0 & 1 & 0 \\
   \end{array} \right\rbrack

Using Custom Factors
~~~~~~~~~~~~~~~~~~~~~~~~

The following C++ code fragment illustrates how to create and add custom
factors to a factor graph:

::

    // add unary measurement factors, like GPS, on all three poses
    noiseModel::Diagonal::shared_ptr unaryNoise =
     noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
    graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
    graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
    graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

In Listing `3.3 <#listing_LocalizationExample2>`__, we create the noise
model on line 2-3, which now specifies two standard deviations on the
measurements :math:`m_{x}` and :math:`m_{y}`. On lines 4-6 we create
***shared\_ptr*** versions of three newly created ***UnaryFactor***
instances, and add them to graph. GTSAM uses shared pointers to refer to
factors in factor graphs, and ***boost::make\_shared*** is a convenience
function to simultaneously construct a class and create a
***shared\_ptr*** to it. We obtain the factor graph from Figure
`4 <#fig_LocalizationFG>`__.

Full Posterior Inference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The three GPS factors are enough to fully constrain all unknown poses
and tie them to a “global” reference frame, including the three unknown
orientations. If not, GTSAM would have exited with a singular matrix
exception. The marginals can be recovered exactly as in Section
`2.5 <#subsec_Full_Posterior_Inference>`__, and the solution and
marginal covariances are now given by the following:

::

    Final Result:
    Values with 3 values:
    Value 1: (-1.5e-14, 1.3e-15, -1.4e-16)
    Value 2: (2, 3.1e-16, -8.5e-17)
    Value 3: (4, -6e-16, -8.2e-17)

    x1 covariance:
          0.0083      4.3e-19     -1.1e-18
         4.3e-19       0.0094      -0.0031
        -1.1e-18      -0.0031       0.0082
    x2 covariance:
          0.0071      2.5e-19     -3.4e-19
         2.5e-19       0.0078      -0.0011
        -3.4e-19      -0.0011       0.0082
    x3 covariance:
         0.0083     4.4e-19     1.2e-18
        4.4e-19      0.0094      0.0031
        1.2e-18      0.0031       0.018

Comparing this with the covariance matrices in Section
`2.5 <#subsec_Full_Posterior_Inference>`__, we can see that the
uncertainty no longer grows without bounds as measurement uncertainty
accumulates. Instead, the “GPS” measurements more or less constrain the
poses evenly, as expected.

|image: 6\_Users\_dellaert\_git\_github\_doc\_images\_Odometry.png|

Sub-Figure a: Odometry marginals

Figure 5: Comparing the marginals resulting from the “odometry” factor
graph in Figure `3 <#fig_OdometryFG>`__ and the “localization” factor
graph in Figure `4 <#fig_LocalizationFG>`__.

|image: 7\_Users\_dellaert\_git\_github\_doc\_images\_Localization.png|

Sub-Figure b: Localization Marginals

It helps a lot when we view this graphically, as in Figure
`5 <#fig_CompareMarginals>`__, where I show the marginals on position as
covariance ellipses that contain 68.26% of all probability mass. For the
odometry marginals, it is immediately apparent from the figure that (1)
the uncertainty on pose keeps growing, and (2) the uncertainty on
angular odometry translates into increasing uncertainty on y. The
localization marginals, in contrast, are constrained by the unary
factors and are all much smaller. In addition, while less apparent, the
uncertainty on the middle pose is actually smaller as it is constrained
by odometry from two sides.

You might now be wondering how we produced these figures. The answer is
via the MATLAB interface of GTSAM, which we will demonstrate in the next
section.
