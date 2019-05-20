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

/** Displays usage information */
void usage() {
  cerr << "wrap parses an interface file and produces a MATLAB or Cython toolbox" << endl;
  cerr << "usage: wrap [--matlab|--cython] interfacePath moduleName toolboxPath cythonImports" << endl;
  cerr << "  interfacePath : *absolute* path to directory of module interface file" << endl;
  cerr << "  moduleName    : the name of the module, interface file must be called moduleName.h" << endl;
  cerr << "  toolboxPath   : the directory in which to generate the wrappers" << endl;
  cerr << "  cythonImports : extra imports for Cython pxd header file" << endl;
}

/**
 * Top-level function to wrap a module
 * @param language can be "--matlab" or "--cython"
 * @param interfacePath path to where interface file lives, e.g., borg/gtsam
 * @param moduleName name of the module to be generated e.g. gtsam
 * @param toolboxPath path where the toolbox should be generated, e.g. borg/gtsam/build
 * @param headerPath is the path to matlab.h
 * @param cythonImports additional imports to include in the generated Cython pxd header file
 */
void generate_toolbox(
           const string& language,
           const string& interfacePath,
           const string& moduleName,
           const string& toolboxPath,
           const string& cythonImports)
{
  // Parse interface file into class object
  // This recursively creates Class objects, Method objects, etc...
  wrap::Module module(interfacePath, moduleName, false);

  if (language == "--matlab")
  // Then emit MATLAB code
    module.generate_matlab_wrapper(toolboxPath);
  else if (language == "--cython") {
    module.generate_cython_wrapper(toolboxPath, cythonImports);
  }
  else {
      cerr << "First argument invalid" << endl;
      cerr << endl;
      usage();
  }
}

/**
 * main parses arguments and calls generate_matlab_toolbox above
 * Typically called from "make all" using appropriate arguments
 */
int main(int argc, const char* argv[]) {
  if (argc != 6) {
    cerr << "Invalid arguments:\n";
    for (int i=0; i<argc; ++i)
      cerr << argv[i] << endl;
    cerr << endl;
    usage();
  }
  else {
    try {
        generate_toolbox(argv[1], argv[2],argv[3],argv[4],argv[5]);
    } catch(std::exception& e) {
      cerr << e.what() << endl;
      return 1;
    }
  }
  return 0;
}
