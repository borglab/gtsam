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

#include <vector>
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
void Class::makefile_fragment(ofstream& ofs) {
//	new_Point2_.$(MEXENDING): new_Point2_.cpp
//		$(MEX) $(mex_flags) new_Point2_.cpp
//	new_Point2_dd.$(MEXENDING): new_Point2_dd.cpp
//		$(MEX) $(mex_flags) new_Point2_dd.cpp
//	@Point2/x.$(MEXENDING): @Point2/x.cpp
//		$(MEX) $(mex_flags) @Point2/x.cpp -output @Point2/x
//	@Point2/y.$(MEXENDING): @Point2/y.cpp
//		$(MEX) $(mex_flags) @Point2/y.cpp -output @Point2/y
//	@Point2/dim.$(MEXENDING): @Point2/dim.cpp
//		$(MEX) $(mex_flags) @Point2/dim.cpp -output @Point2/dim
//
//	Point2: new_Point2_.$(MEXENDING) new_Point2_dd.$(MEXENDING) @Point2/x.$(MEXENDING) @Point2/y.$(MEXENDING) @Point2/dim.$(MEXENDING)

	// collect names
	vector<string> file_names;
  BOOST_FOREACH(Constructor c, constructors) {
  	string file_base = c.matlab_wrapper_name(name);
  	file_names.push_back(file_base);
  }
  BOOST_FOREACH(StaticMethod c, static_methods) {
  	string file_base = name + "_" + c.name_;
  	file_names.push_back(file_base);
  }
  BOOST_FOREACH(Method c, methods) {
  	string file_base = "@" + name + "/" + c.name_;
  	file_names.push_back(file_base);
  }

  BOOST_FOREACH(const string& file_base, file_names) {
  	ofs << file_base << ".$(MEXENDING): " << file_base << ".cpp" << endl;
  	ofs << "\t$(MEX) $(mex_flags) " << file_base << ".cpp  -output " << file_base << endl;
  }

	// class target
  ofs << "\n" << name << ": ";
  BOOST_FOREACH(const string& file_base, file_names) {
    	ofs << file_base << ".$(MEXENDING) ";
  }
  ofs << "\n" << endl;
}

/* ************************************************************************* */
