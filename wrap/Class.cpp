/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.cpp
 * @author Frank Dellaert
 * @author Andrew Melim
 **/

#include <vector>
#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include "Class.h"
#include "utilities.h"
#include "Argument.h"

using namespace std;
using namespace wrap;

static const uint64_t ptr_constructor_key =
	(uint64_t('G') << 56) |
	(uint64_t('T') << 48) |
	(uint64_t('S') << 40) |
	(uint64_t('A') << 32) |
	(uint64_t('M') << 24) |
	(uint64_t('p') << 16) |
	(uint64_t('t') << 8) |
	(uint64_t('r'));

/* ************************************************************************* */
void Class::matlab_proxy(const string& classFile, const string& wrapperName, FileWriter& wrapperFile, vector<string>& functionNames) const {
  // open destination classFile
  FileWriter proxyFile(classFile, verbose_, "%");

  // get the name of actual matlab object
  string matlabName = qualifiedName(), cppName = qualifiedName("::");

  // emit class proxy code
  // we want our class to inherit the handle class for memory purposes
  proxyFile.oss << "classdef " << matlabName << " < handle" << endl;
  proxyFile.oss << "  properties" << endl;
  proxyFile.oss << "    self = 0" << endl;
  proxyFile.oss << "  end" << endl;
  proxyFile.oss << "  methods" << endl;

  // Constructor
  proxyFile.oss << "    function obj = " << matlabName << "(varargin)" << endl;
  // Special pointer constructor
	{
		proxyFile.oss << "      if nargin == 2 && isa(varargin{1}, 'uint64') && ";
		proxyFile.oss << "varargin{1} == uint64(" << ptr_constructor_key << ")\n";
		proxyFile.oss << "        obj.self = varargin{2};\n";
	}
	// Regular constructors
  BOOST_FOREACH(ArgumentList a, constructor.args_list)
  {
		const int id = functionNames.size();
    constructor.proxy_fragment(proxyFile, wrapperName, matlabName, id, a);
		const string wrapFunctionName = constructor.wrapper_fragment(wrapperFile,
			cppName, matlabName, id, using_namespaces, includes, a);
		wrapperFile.oss << "\n";
    functionNames.push_back(wrapFunctionName);
  }
  proxyFile.oss << "      else\n";
	proxyFile.oss << "        error('" << matlabName << " constructor failed');" << endl;
	proxyFile.oss << "      end\n";
  proxyFile.oss << "    end\n\n";

	// Deconstructor
	{
		const int id = functionNames.size();
		deconstructor.proxy_fragment(proxyFile, wrapperName, matlabName, id);
		proxyFile.oss << "\n";
		const string functionName = deconstructor.wrapper_fragment(wrapperFile,
			cppName, matlabName, id, using_namespaces, includes);
		wrapperFile.oss << "\n";
		functionNames.push_back(functionName);
	}
  proxyFile.oss << "    function display(obj), obj.print(''); end\n\n";
  proxyFile.oss << "    function disp(obj), obj.display; end\n\n";

	// Methods
	BOOST_FOREACH(Method m, methods) {
		const int id = functionNames.size();
		m.proxy_fragment(proxyFile, wrapperName, id);
		proxyFile.oss << "\n";
		const string wrapFunctionName = m.wrapper_fragment(wrapperFile,
			cppName, matlabName, id, using_namespaces);
		wrapperFile.oss << "\n";
		functionNames.push_back(wrapFunctionName);
	}

	proxyFile.oss << "  end" << endl;
	proxyFile.oss << "end" << endl;

  // Close file
  proxyFile.emit(true);
}

/* ************************************************************************* */
void Class::matlab_static_methods(const string& toolboxPath, const string& wrapperName,
																	FileWriter& wrapperFile, vector<string>& functionNames) const {
	string matlabName = qualifiedName(), cppName = qualifiedName("::");
  BOOST_FOREACH(const StaticMethod& m, static_methods) {
		const int id = functionNames.size();
		m.proxy_fragment(toolboxPath, matlabName, wrapperName, id);
    const string wrapFunction = m.wrapper_fragment(wrapperFile, matlabName, cppName, id, using_namespaces);
		functionNames.push_back(wrapFunction);
  }
}

/* ************************************************************************* */
string Class::qualifiedName(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces)
		result += ns + delim;
	return result + name;
}

/* ************************************************************************* */
