# WRAP README

The wrap library wraps the GTSAM library into a MATLAB toolbox. 

It was designed to be more general than just wrapping GTSAM, but a small amount of GTSAM specific code exists in matlab.h, the include file that is included by the mex files. The GTSAM-specific functionality consists primarily of handling of Eigen Matrix and Vector classes.  

For notes on creating a wrap interface, see gtsam.h for what features can be wrapped into a toolbox, as well as the current state of the toolbox for gtsam. For more technical details on the interface, please read comments in matlab.h

Some good things to know:

OBJECT CREATION

- Classes are created by special constructors, e.g., new_GaussianFactorGraph_.cpp.
	These constructors are called from the MATLAB class @GaussianFactorGraph.
	new_GaussianFactorGraph_ calls wrap_constructed in matlab.h, see documentation there
	
METHOD (AND CONSTRUCTOR) ARGUMENTS

- Simple argument types of methods, such as "double", will be converted in the
  mex wrappers by calling unwrap<double>, defined in matlab.h
- Vector and Matrix arguments are normally passed by reference in GTSAM, but
  in gtsam.h you need to pretend they are passed by value, to trigger the 
  generation of the correct conversion routines unwrap<Vector> and unwrap<Matrix>
- passing classes as arguments works, provided they are passed by reference.
	This triggers a call to unwrap_shared_ptr

   