/**
 * file: Class.ccp
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "Class.h"
#include "utilities.h"

using namespace std;

/* ************************************************************************* */
void Class::matlab_proxy(const string& classFile) {
  // open destination classFile
  ofstream ofs(classFile.c_str());
  if(!ofs) throw CantOpenFile(classFile);
  cerr << "generating " << classFile << endl;

  // emit class proxy code
  emit_header_comment(ofs,"%");
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
  ofs << "    function display(obj), obj.print; end" << endl;
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
void Class::matlab_make_fragment(ofstream& ofs, 
				 const string& toolboxPath,
				 const string& mexFlags) 
{
  string mex = "mex " + mexFlags + " ";
  BOOST_FOREACH(Constructor c, constructors)
    ofs << mex << c.matlab_wrapper_name(name) << ".cpp" << endl;
  ofs << endl << "cd @" << name << endl;
  BOOST_FOREACH(Method m, methods)
    ofs << mex << m.name << ".cpp" << endl;
  ofs << endl;
}

/* ************************************************************************* */
