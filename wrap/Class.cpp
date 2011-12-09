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

  // get the name of actual matlab object
  string matlabName = qualifiedName();

  // emit class proxy code
  ofs << "classdef " << matlabName << endl;
  ofs << "  properties" << endl;
  ofs << "    self = 0" << endl;
  ofs << "  end" << endl;
  ofs << "  methods" << endl;
  ofs << "    function obj = " << matlabName << "(varargin)" << endl;
  BOOST_FOREACH(Constructor c, constructors)
    c.matlab_proxy_fragment(ofs,matlabName);
  ofs << "      if nargin ~= 13 && obj.self == 0, error('" << matlabName << " constructor failed'); end" << endl;
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
    c.matlab_mfile  (toolboxPath, qualifiedName());
    c.matlab_wrapper(toolboxPath, qualifiedName("::"), qualifiedName(), nameSpace, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_methods(const string& classPath, const string& nameSpace) {
	string matlabName = qualifiedName(), cppName = qualifiedName("::");
  BOOST_FOREACH(Method m, methods) {
    m.matlab_mfile  (classPath);
    m.matlab_wrapper(classPath, name, cppName, matlabName, nameSpace, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_static_methods(const string& toolboxPath, const string& nameSpace) {
	string matlabName = qualifiedName(), cppName = qualifiedName("::");
  BOOST_FOREACH(StaticMethod& m, static_methods) {
    m.matlab_mfile  (toolboxPath, qualifiedName());
    m.matlab_wrapper(toolboxPath, name, matlabName, cppName, nameSpace, includes);
  }
}

/* ************************************************************************* */
void Class::matlab_make_fragment(ofstream& ofs, 
				 const string& toolboxPath,
				 const string& mexFlags) 
{
  string mex = "mex " + mexFlags + " ";
  string matlabClassName = qualifiedName();
  BOOST_FOREACH(Constructor c, constructors)
    ofs << mex << c.matlab_wrapper_name(matlabClassName) << ".cpp" << endl;
  BOOST_FOREACH(StaticMethod sm, static_methods)
    ofs << mex << matlabClassName + "_" + sm.name << ".cpp" << endl;
  ofs << endl << "cd @" << matlabClassName << endl;
  BOOST_FOREACH(Method m, methods)
    ofs << mex << m.name << ".cpp" << endl;
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

	string matlabName = qualifiedName();

	// collect names
	vector<string> file_names;
  BOOST_FOREACH(Constructor c, constructors) {
  	string file_base = c.matlab_wrapper_name(matlabName);
  	file_names.push_back(file_base);
  }
  BOOST_FOREACH(StaticMethod c, static_methods) {
  	string file_base = matlabName + "_" + c.name;
  	file_names.push_back(file_base);
  }
  BOOST_FOREACH(Method c, methods) {
  	string file_base = "@" + matlabName + "/" + c.name;
  	file_names.push_back(file_base);
  }

  BOOST_FOREACH(const string& file_base, file_names) {
  	ofs << file_base << ".$(MEXENDING): " << file_base << ".cpp" << endl;
  	ofs << "\t$(MEX) $(mex_flags) " << file_base << ".cpp  -output " << file_base << endl;
  }

	// class target
  ofs << "\n" << matlabName << ": ";
  BOOST_FOREACH(const string& file_base, file_names) {
    	ofs << file_base << ".$(MEXENDING) ";
  }
  ofs << "\n" << endl;
}

/* ************************************************************************* */
string Class::qualifiedName(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces)
		result += ns + delim;
	return result + name;
}

/* ************************************************************************* */
