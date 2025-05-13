# easyslam Library Documentation

_Alireza Fathi_

## Marker

This class understands that the markers are square, and they have four corners. It knows what the order of corners is.

- hGetP: returns the pixel location of a particular corner of the marker in the image, given the marker and camera pose.
- hGetAll: returns the pixel locations of all corners of the marker in a vector, given the marker and camera pose.
- DhGetP_pose: returns a matrix which is the derivative of the change in the pixel location of a particular corner, with respect to the camera pose.
- DhGetP_marker: returns a matrix which is the derivative of the change in the pixel location of a particular corner, with respect to the maker pose.
- DhGetP: returns derivative matrices of the change in the pixel location of a particular corner, with respect to the maker pose and camera pose.

## ARRobotMarker

The robot sees a bunch of markers in each frame. The ARRobotMarker file is used to keep the information gathered from each observation such as: the frame number (robotNumber), the marker ID (markerNumber), the x/y pixel location of the four corners of the marker (Points), size of the marker in milimeters (markerSize_) and the camera calibration(K_).

**ARRobotMarker is the ugly class in here. It has at least three dangerous things happening which I describe below**:

- **Hack1**: This class has a constructor that receives nothing. This constructor should never get called. It sets variables to nonsense values.

- **Hack**2: There is a variable (estimatedPose_) which keeps the relative pose of the marker to the camera. At the beginning that the ARRobotMarker is created (using the constructor that takes values), the estimate_pose() function is called. This function computes the pose of the marker with respect to the robot, using homography. The homography doesn't return a precise answer but is a good way to initialize the relative pose. The reason that I have this variable is that I first compute it using homography, but the developer can have other methods for optimizing this relative pose. For example in EasySLAM project, in ARFixer class, I have a function (optimizedTransformation()) that uses a factor graph, to optimize the relative pose. In that function the factor graph has only two factors, one unary factor to set the camera to the origin, and one binary factor between the camera and marker.

- **Hack3**: fixOrderingOfPoints() is such a hack! The reason to have this function is because of the following observation. I realized that ARToolKit always returns the points in a clockwise order, but does not always start from the bottom left corner of the marker. (maybe this was a bug in my dumping the codes in ARToolkit). I fix this ordering, bu assuming the robot is always on the ground and the markers are stuck to the wall in a way that their bottom left corner is in the bottom left. So If the ARToolkit doesn't return the corner that appears in the bottom left of the image first, I fix the ordering to satisfy this rule. (This works even if marker is not stuck the way it is supposed to, since the point is that different observations be consistent over the time).

- **Hack4**: The other thing which might not be a hack though is that I make sure that the ARToolKit is not giving me nonsense data. (which it does sometime!? again this might be a bug in my usage of ARToolKit). I look at the points and make sure the marker is appearing as a convex shape in the image. Also I make sure the area of the markers is bigger than a threshod.

**Note that the stuff described in Hack3 and Hack4, are not applied in ARRobotMarker when it loads the markers from files, I have these stuff in ARFixer in EasySLAM project.**

- loadARToolKit: loads the the information from a path and puts it in a vector of ARRobotMarker.
- loadARToolKit_oneFrame: just loads the information of a particular frame. It is used when we read one frame at a time.
- saveARToolKit: saves the information to a path.
- dumpAR: dumps the information to a file. The difference between saveARToolKit and dumpAR is that, dumpAR puts everything in one file, whicl saveARToolKit puts the information into the format we read from.
- load_dumpedAR: loads from the dumped file.
- getReferenceMarker: in an image returns the biggest marker as the reference marker, if the previousMarker input variable is set to -1. Otherwise, it knows the ID of the previous marker, and if it exists in this frame as well, returns that as the reference marker. This might be good because we do not want to change the reference frame very frequently.

## utility.h/homography.h

These files are well documented, and they are low level. So please refer to the .h files for the documentation.

## EasySLAMGraph

It is the child of NonlinearFactorGraph (in gtsam). EasySLAMGraph is a nonlinear factor graph, and has two kinds of factors. It has unary factors of the CameraMarkerFactor0 type, and binary factors of the type CameraMarkerFactor. CameraMarkerFactor0 is a unary factor containing the robot pose, by assuming the marker being at the origin. The EasySLAMGraph sets the referenceMarker to the origin. A unary facto is created everytime the robot sees the reference marker. CameraMarkerFactor is a binary factor between the camera and marker.

- EasySLAMGraph (constructor): loads nrFrames (starting from frame 1), from the files in path. Calibration and marker size are among the information that are saved in files in the path. Also the pixel location of the marker corners are in files. It sets the reference marker to the first marker seen.
- load: adds markers to the factor graph.
- loadAFrame: read the information of one frame from the path, and adds it to the factor graph.
- loadAFrame: Christian has added this! it is just an alias! why?
- insertMarker: Christian has added this. Why?
- dump/load_dumped: To save the graph in a file or read it from a file.

## EasySLAMConfig

It contains the pose of the markers (markerPoses) and the pose of the robot i.e. the camera (robotPoses).

- EasySLAMConfig: initializes the config. What does that mean? It means that it loads the vector of ARRobotMarkers, which contains the observations, and initializes the config given these observations. Initialization is performed by building a spanning tree as described in the paper.
- EasySLAMConfig: this constructor receives a FGConfig and changes it to EasySLAMConfig. The assumption is that the FGConfig in in the supposed format.
- getFGConfig: returns the FGConfig version of the EasySLAMConfig.
- load: is used to initialize the config from the observations.
- loadAFrame: loads one frame.
- loadAFrame: added by Christian. why?
- flush/dump/load_dumped: reading and writing from/to file(s).
- transform_to: I think it is written by Christian. It transforms everything to new coordinate frame, which its origin is the input pose. (This is my guess)

## CameraMarkerFactor

It is a binary non linear factor between a robot and a marker. I mean it is the factor created for the image of the marker in a frame.

## CameraMarkerFactor0

It is a unary non linear factor that forces the marker to be at the origin and sets the robot to be in a particular locations, based on its relative pose to the marker.