/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file wrap.cpp
 * @brief wraps functions
 * @author Frank Dellaert
 **/

#include <stdio.h>
#include <iostream>

#include "Module.h"

using namespace std;

/**
 * Top-level function to wrap a module
 * @param mexCommand is a sufficiently qualified command to execute mex within a makefile
 * @param mexExt is the extension for mex binaries for this os/cpu
 * @param interfacePath path to where interface file lives, e.g., borg/gtsam
 * @param moduleName name of the module to be generated e.g. gtsam
 * @param toolboxPath path where the toolbox should be generated, e.g. borg/gtsam/build
 * @param nameSpace e.g. gtsam
 * @param mexFlags extra arguments for mex script, i.e., include flags etc...
 */
void generate_matlab_toolbox(
					 const string& mexCommand,
					 const string& mexExt,
					 const string& interfacePath,
			     const string& moduleName,
			     const string& toolboxPath,
			     const string& headerPath,
			     const string& mexFlags)
{
  // Parse interface file into class object
	// This recursively creates Class objects, Method objects, etc...
  wrap::Module module(interfacePath, moduleName, false);

  // Then emit MATLAB code
  module.matlab_code(mexCommand,toolboxPath,mexExt,headerPath,mexFlags);
}

/** Displays usage information */
void usage() {
  cerr << "wrap parses an interface file and produces a MATLAB toolbox" << endl;
  cerr << "usage: wrap mexExecutable mexExtension interfacePath moduleName toolboxPath [mexFlags]" << endl;
  cerr << "  mexExecutable : command to execute mex if on path, use 'mex'" << endl;
  cerr << "  mexExtension  : OS/CPU-dependent extension for MEX binaries" << endl;
  cerr << "  interfacePath : *absolute* path to directory of module interface file" << endl;
  cerr << "  moduleName    : the name of the module, interface file must be called moduleName.h" << endl;
  cerr << "  toolboxPath   : the directory in which to generate the wrappers" << endl;
  cerr << "  [mexFlags]    : extra flags for the mex command" << endl;
}

/**
 * main parses arguments and calls generate_matlab_toolbox above
 * Typically called from "make all" using appropriate arguments
 */
int main(int argc, const char* argv[]) {
  if (argc<7 || argc>8) {
  	cerr << "Invalid arguments:\n";
  	for (int i=0; i<argc; ++i)
  		cerr << argv[i] << endl;
  	cerr << endl;
  	usage();
  }
  else
    generate_matlab_toolbox(argv[1],argv[2],argv[3],argv[4],argv[5],argv[6],argc==7 ? " " : argv[7]);
  return 0;
}
