/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.ccp
 * @author Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "Class.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Class::matlab_proxy(const string& classFile) {
  // open destination classFile
  ofstream ofs(classFile.c_str());
  if(!ofs) throw CantOpenFile(classFile);
  if(verbose_) cerr << "generating " << classFile << endl;

  // emit class proxy code
  ofs << "classdef " << name << endl;
  ofs << "  properties" << endl;
  ofs << "    self = 0" << endl;
  ofs << "  end" << endl;
  ofs << "  methods" << endl;
  ofs << "    function obj = " << name << "(varargin)" << endl;
  BOOST_FOREACH(Constructor c, constructors)
    c.matlab_proxy_fragment(ofs,name);
  ofs << "      if nargin ~= 13 && obj.self == 0, error('" << name << " constructor failed'); end" << endl;
  ofs << "    end" << endl;
  ofs << "    function display(obj), obj.print(''); end" << endl;
  ofs << "    function disp(obj), obj.display; end" << endl;
  ofs << "  end" << endl;
  ofs << "end" << endl;

  // close file
  ofs.close();
}

/* ************************************************************************* */
void Class::matlab_constructors(const string& toolboxPath,const string& nameSpace) {
  BOOST_FOREACH(Constructor c, constructors) {
    c.matlab_mfile  (toolboxPath, name);
    c.matlab_wrapper(toolboxPath, name, nameSpace);
  }
}

/* ************************************************************************* */
void Class::matlab_methods(const string& classPath, const string& nameSpace) {
  BOOST_FOREACH(Method m, methods) {
    m.matlab_mfile  (classPath);
    m.matlab_wrapper(classPath, name, nameSpace);
  }
}

/* ************************************************************************* */
void Class::matlab_static_methods(const string& toolboxPath, const string& nameSpace) {
  BOOST_FOREACH(StaticMethod& m, static_methods) {
    m.matlab_mfile  (toolboxPath, name);
    m.matlab_wrapper(toolboxPath, name, nameSpace);
  }
}

/* ************************************************************************* */
void Class::matlab_make_fragment(ofstream& ofs, 
				 const string& toolboxPath,
				 const string& mexFlags) 
{
  string mex = "mex " + mexFlags + " ";
  BOOST_FOREACH(Constructor c, constructors)
    ofs << mex << c.matlab_wrapper_name(name) << ".cpp" << endl;
  BOOST_FOREACH(StaticMethod sm, static_methods)
    ofs << mex << name + "_" + sm.name_ << ".cpp" << endl;
  ofs << endl << "cd @" << name << endl;
  BOOST_FOREACH(Method m, methods)
    ofs << mex << m.name_ << ".cpp" << endl;
  ofs << endl;
}

/* ************************************************************************* */
