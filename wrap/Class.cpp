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
void Class::matlab_proxy(const string& classFile) const {
  // open destination classFile
  FileWriter file(classFile, verbose_, "%");

  // get the name of actual matlab object
  string matlabName = qualifiedName();

  // emit class proxy code
  // we want our class to inherit the handle class for memory purposes
  file.oss << "classdef " << matlabName << " < handle" << endl;
  file.oss << "  properties" << endl;
  file.oss << "    self = 0" << endl;
  file.oss << "  end" << endl;
  file.oss << "  methods" << endl;
  // constructor
  file.oss << "    function obj = " << matlabName << "(varargin)" << endl;
  BOOST_FOREACH(Constructor c, constructors)
    c.matlab_proxy_fragment(file,matlabName);
  file.oss << "      if nargin ~= 13 && obj.self == 0, error('" << matlabName << " constructor failed'); end" << endl;
  file.oss << "    end" << endl;
  // deconstructor
  file.oss << "    function delete(obj)" << endl;
  file.oss << "       delete_" << matlabName << "(obj);" << endl;
  file.oss << "    end" << endl;
  file.oss << "    function display(obj), obj.print(''); end" << endl;
  file.oss << "    function disp(obj), obj.display; end" << endl;
  file.oss << "  end" << endl;
  file.oss << "end" << endl;

  // close file
  file.emit(true);
}

/* ************************************************************************* */
void Class::matlab_constructors(const string& toolboxPath) const {
  BOOST_FOREACH(Constructor c, constructors) {
    c.matlab_mfile  (toolboxPath, qualifiedName());
    c.matlab_wrapper(toolboxPath, qualifiedName("::"), qualifiedName(), using_namespaces, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_deconstructor(const string& toolboxPath) const {
    d.matlab_mfile  (toolboxPath, qualifiedName());
    d.matlab_wrapper(toolboxPath, qualifiedName("::"), qualifiedName(), using_namespaces, includes);
}
/* ************************************************************************* */
void Class::matlab_methods(const string& classPath) const {
	string matlabName = qualifiedName(), cppName = qualifiedName("::");
  BOOST_FOREACH(Method m, methods) {
    m.matlab_mfile  (classPath);
    m.matlab_wrapper(classPath, name, cppName, matlabName, using_namespaces, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_static_methods(const string& toolboxPath) const {
	string matlabName = qualifiedName(), cppName = qualifiedName("::");
  BOOST_FOREACH(const StaticMethod& m, static_methods) {
    m.matlab_mfile  (toolboxPath, qualifiedName());
    m.matlab_wrapper(toolboxPath, name, matlabName, cppName, using_namespaces, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_make_fragment(FileWriter& file, 
				 const string& toolboxPath,
				 const string& mexFlags) const {
  string mex = "mex " + mexFlags + " ";
  string matlabClassName = qualifiedName();
  BOOST_FOREACH(Constructor c, constructors)
    file.oss << mex << c.matlab_wrapper_name(matlabClassName) << ".cpp" << endl;
  file.oss << mex << d.matlab_wrapper_name(matlabClassName) << ".cpp" << endl;
  BOOST_FOREACH(StaticMethod sm, static_methods)
    file.oss << mex << matlabClassName + "_" + sm.name << ".cpp" << endl;
  file.oss << endl << "cd @" << matlabClassName << endl;
  BOOST_FOREACH(Method m, methods)
    file.oss << mex << m.name << ".cpp" << endl;
  file.oss << endl;
}

/* ************************************************************************* */
void Class::makefile_fragment(FileWriter& file) const {
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

	string matlabName = qualifiedName();

	// collect names
	vector<string> file_names;
  BOOST_FOREACH(Constructor c, constructors) {
  	string file_base = c.matlab_wrapper_name(matlabName);
  	file_names.push_back(file_base);
  }
  file_names.push_back(d.matlab_wrapper_name(matlabName));
  BOOST_FOREACH(StaticMethod c, static_methods) {
  	string file_base = matlabName + "_" + c.name;
  	file_names.push_back(file_base);
  }
  BOOST_FOREACH(Method c, methods) {
  	string file_base = "@" + matlabName + "/" + c.name;
  	file_names.push_back(file_base);
  }

  BOOST_FOREACH(const string& file_base, file_names) {
  	file.oss << file_base << ".$(MEXENDING): " << file_base << ".cpp" << endl;
  	file.oss << "\t$(MEX) $(mex_flags) " << file_base << ".cpp  -output " << file_base << endl;
  }

	// class target
  file.oss << "\n" << matlabName << ": ";
  BOOST_FOREACH(const string& file_base, file_names) {
    	file.oss << file_base << ".$(MEXENDING) ";
  }
  file.oss << "\n" << endl;
}

/* ************************************************************************* */
string Class::qualifiedName(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces)
		result += ns + delim;
	return result + name;
}

/* ************************************************************************* */
