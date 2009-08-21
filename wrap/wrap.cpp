/**
 * file: wrap.ccp
 * brief: wraps functions
 * Author: Frank Dellaert
 **/

#include <stdio.h>
#include <iostream>

#include "Module.h"

using namespace std;

/* ************************************************************************* */
/**
 * main function to wrap a module
 */
void generate_matlab_toolbox(const string& interfacePath,
			     const string& moduleName,
			     const string& toolboxPath,
			     const string& nameSpace,
			     const string& mexFlags) 
{
  // Parse into class object
  Module module(interfacePath, moduleName);

  // emit MATLAB code
  module.matlab_code(toolboxPath,nameSpace,mexFlags);
}

/* ************************************************************************* */

int main(int argc, const char* argv[]) {
  if (argc<5 || argc>6) {
    cerr << "wrap parses an interface file and produces a MATLAB toolbox" << endl;
    cerr << "usage: wrap interfacePath moduleName toolboxPath" << endl;
    cerr << "  interfacePath : *absolute* path to directory of module interface file" << endl;
    cerr << "  moduleName    : the name of the module, interface file must be called moduleName.h" << endl;
    cerr << "  toolboxPath   : the directory in which to generate the wrappers" << endl;
    cerr << "  nameSpace     : namespace to use, pass empty string if none" << endl;
    cerr << "  [mexFlags]    : extra flags for the mex command" << endl;
  }
  else
    generate_matlab_toolbox(argv[1],argv[2],argv[3],argv[4],argc==5 ? " " : argv[5]);
}

/* ************************************************************************* */
