Structure from Motion
-----------------------

|image: 16\_Users\_dellaert\_git\_github\_doc\_images\_cube.png| Figure
14: An optimized “Structure from Motion” with 10 cameras arranged in a
circle, observing the 8 vertices of a :math:`20 \times 20 \times 20`
cube centered around the origin. The camera is rendered with color-coded
axes, (RGB for XYZ) and the viewing direction is is along the positive
Z-axis. Also shown are the 3D error covariance ellipses for both cameras
and points.

**Structure from Motion** (SFM) is a technique to recover a 3D
reconstruction of the environment from corresponding visual features in
a collection of *unordered* images, see Figure `14 <#fig_SFMExample>`__.
In GTSAM this is done using exactly the same factor graph framework,
simply using SFM-specific measurement factors. In particular, there is a
**projection factor** that calculates the reprojection error
:math:`f\left( {x_{i},p_{j};z_{ij},K} \right)` for a given camera pose
:math:`x_{i}` (a ***Pose3***) and point :math:`p_{j}` (a ***Point3***).
The factor is parameterized by the 2D measurement :math:`z_{ij}` (a
***Point2***), and known calibration parameters :math:`K` (of type
***Cal3\_S2***). The following listing shows how to create the factor
graph:
::

    %% Add factors for all measurements
    noise = noiseModel.Isotropic.Sigma(2, measurementNoiseSigma);
    for i = 1:length(Z),
        for k = 1:length(Z{i})
            j = J{i}{k};
            G.add(GenericProjectionFactorCal3_S2(
                  Z{i}{k}, noise, symbol('x', i), symbol('p', j), K));
        end
    end

In Listing `6 <#listing_SFMExample>`__, assuming that the factor graph
was already created, we add measurement factors in the double loop. We
loop over images with index :math:`i`, and in this example the data is
given as two cell arrays: Z{i} specifies a set of measurements
:math:`z_{k}` in image :math:`i`, and J{i} specifies the corresponding
point index. The specific factor type we use is a
***GenericProjectionFactorCal3\_S2***, which is the MATLAB equivalent of
the C++ class ***GenericProjectionFactor<Cal3\_S2>***, where
***Cal3\_S2*** is the camera calibration type we choose to use (the
standard, no-radial distortion, 5 parameter calibration matrix). As
before landmark-based SLAM (Section `5 <#sec_Landmark_based_SLAM>`__),
here we use symbol keys except we now use the character 'p' to denote
points, rather than 'l' for landmark.

Important note: a very tricky and difficult part of making SFM work is
(a) data association, and (b) initialization. GTSAM does neither of
these things for you: it simply provides the “bundle adjustment”
optimization. In the example, we simply assume the data association is
known (it is encoded in the J sets), and we initialize with the ground
truth, as the intent of the example is simply to show you how to set up
the optimization problem.
