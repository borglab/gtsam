# GTSAM Examples

This directory contains all GTSAM C++ examples GTSAM pertaining to SFM


## Basic Examples:

* **SimpleRotation**:  a simple example of optimizing a single rotation according to a single prior
* **CameraResectioning**: resection camera from some known points
* **SFMExample**: basic structure from motion
* **SFMExample_bal**: same, but read data from read from BAL file
* **SelfCalibrationExample**: Do SFM while also optimizing for calibration

## Stereo Visual Odometry Examples
Visual odometry using a stereo rig:

* **StereoVOExample**: basic example of stereo VO
* **StereoVOExample_large**: larger, with a snippet of Kitti data

## More Advanced Examples
The following examples illustrate some concepts from Georgia Tech's research papers, listed in the references section at the end:

* **VisualISAMExample**: uses iSAM [TRO08]
* **VisualISAM2Example**: uses iSAM2 [IJRR12]
* **SFMExample_SmartFactor**: uses smartFactors [ICRA14]

## Kalman Filter Examples
* **elaboratePoint2KalmanFilter**: simple linear Kalman filter on a moving 2D point, but done using factor graphs
* **easyPoint2KalmanFilter**: uses the generic templated Kalman filter class to do the same
* **fullStateKalmanFilter**: simple 1D example with a full-state filter
* **errorStateKalmanFilter**: simple 1D example of a moving target measured by a accelerometer, incl. drift-rate bias

## 2D Pose SLAM 

* **LocalizationExample.cpp**: modeling robot motion
* **LocalizationExample2.cpp**: example with GPS like measurements
* **Pose2SLAMExample**: A 2D Pose SLAM example using the predefined typedefs in gtsam/slam/pose2SLAM.h
* **Pose2SLAMExample_advanced**: same, but uses an Optimizer object
* **Pose2SLAMwSPCG**: solve a simple 3 by 3 grid of Pose2 SLAM problem by using easy SPCG interface

## Planar SLAM with landmarks
* **PlanarSLAMExample**: simple robotics example using the pre-built planar SLAM domain
* **PlanarSLAMExample_selfcontained**: simple robotics example with all typedefs internal to this script.

## Visual SLAM

The directory **vSLAMexample** includes 2 simple examples using GTSAM:

- **vSFMexample** using visual SLAM for structure-from-motion (SFM)
- **vISAMexample** using visual SLAM and ISAM for incremental SLAM updates

See the separate README file there.

## Undirected Graphical Models (UGM)
The best representation for a Markov Random Field is a factor graph :-) This is illustrated with some discrete examples from the UGM MATLAB toolbox, which
can be found at <http://www.di.ens.fr/~mschmidt/Software/UGM>


## Building and Running
To build, cd into the top-level gtsam directory and do:

```
mkdir build
cd build
cmake ..
```

For each .cpp file in this directory two make targets are created, one to build the executable, and one to build and run it. For example, the file `CameraResectioning.cpp` contains simple example to resection a camera from 4 known points. You can build it using

```
make CameraResectioning
```
or build and run it immediately with

```
make CameraResectioning.run
```
which should output:

```
Final result:
Values with 1 values:
Value x1: R:
[
           1,	         0.0,	         0.0,	
         0.0,	          -1,	         0.0,	
         0.0,	         0.0,	          -1,	
];
t: [0, 0, 2]';
```


## References
- [TRO08]: [iSAM: Incremental Smoothing and Mapping, Michael Kaess](http://frank.dellaert.com/pub/Kaess08tro.pdf), Michael Kaess, Ananth Ranganathan, and Frank Dellaert, IEEE Transactions on Robotics, 2008
- [IJRR12]: [iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree](http://www.cc.gatech.edu/~dellaert/pub/Kaess12ijrr.pdf), Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, and Frank Dellaert, International Journal of Robotics Research, 2012
- [ICRA14]: [Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors](http://frank.dellaert.com/pub/Carlone14icra.pdf), Luca Carlone, Zsolt Kira, Chris Beall, Vadim Indelman, and Frank Dellaert, IEEE International Conference on Robotics and Automation (ICRA), 2014
