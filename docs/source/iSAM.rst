iSAM: Incremental Smoothing and Mapping
-----------------------------------------

GTSAM provides an incremental inference algorithm based on a more
advanced graphical model, the Bayes tree, which is kept up to date by
the **iSAM** algorithm (incremental Smoothing and Mapping, see `Kaess et
al. <#LyXCite-Kaess08tro>`__ (2008); `Kaess et
al. <#LyXCite-Kaess12ijrr>`__ (2012) for an in-depth treatment). For
mobile robots operating in real-time it is important to have access to
an updated map as soon as new sensor measurements come in. iSAM keeps
the map up-to-date in an efficient manner.

Listing `7 <#listing_iSAMExample>`__ shows how to use iSAM in a simple
visual SLAM example. In line 1-2 we create a ***NonlinearISAM*** object
which will relinearize and reorder the variables every 3 steps. The
corect value for this parameter depends on how non-linear your problem
is and how close you want to be to gold-standard solution at every step.
In iSAM 2.0, this parameter is not needed, as iSAM2 automatically
determines when linearization is needed and for which variables.

The example involves eight 3D points that are seen from eight successive
camera poses. Hence in the first step -which is omitted here- all eight
landmarks and the first pose are properly initialized. In the code this
is done by perturbing the known ground truth, but in a real application
great care is needed to properly initialize poses and landmarks,
especially in a monocular sequence.

::

    int relinearizeInterval = 3;
    NonlinearISAM isam(relinearizeInterval);

    // ... first frame initialization omitted ...

    // Loop over the different poses, adding the observations to iSAM
    for (size_t i = 1; i < poses.size(); ++i) {

      // Add factors for each landmark observation
      NonlinearFactorGraph graph;
      for (size_t j = 0; j < points.size(); ++j) {
        graph.add(
          GenericProjectionFactor<Pose3, Point3, Cal3_S2>
            (z[i][j], noise,Symbol('x', i), Symbol('l', j), K)
        );
      }

      // Add an initial guess for the current pose
      Values initialEstimate;
      initialEstimate.insert(Symbol('x', i), initial_x[i]);

      // Update iSAM with the new factors
      isam.update(graph, initialEstimate);
     }

The remainder of the code illustrates a typical iSAM loop:

#. Create factors for new measurements. Here, in lines 9-18, a small
   ***NonlinearFactorGraph*** is created to hold the new factors of type
   ***GenericProjectionFactor<Pose3, Point3, Cal3\_S2>***.
#. Create an initial estimate for all newly introduced variables. In
   this small example, all landmarks have been observed in frame 1 and
   hence the only new variable that needs to be initialized at each time
   step is the new pose. This is done in lines 20-22. Note we assume a
   good initial estimate is available as *initial\_x[i]*.
#. Finally, we call *isam.update()*, which takes the factors and initial
   estimates, and incrementally updates the solution, which is available
   through the method *isam.estimate()*, if desired.
