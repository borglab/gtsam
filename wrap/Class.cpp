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
#include <stdint.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "Class.h"
#include "utilities.h"
#include "Argument.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Class::matlab_proxy(const string& classFile, const string& wrapperName, FileWriter& wrapperFile, vector<string>& functionNames) const {
  // open destination classFile
  FileWriter proxyFile(classFile, verbose_, "%");

  // get the name of actual matlab object
  const string matlabName = qualifiedName(), cppName = qualifiedName("::");

  // emit class proxy code
  // we want our class to inherit the handle class for memory purposes
  proxyFile.oss << "classdef " << matlabName << " < handle" << endl;
  proxyFile.oss << "  properties" << endl;
  proxyFile.oss << "    self = 0" << endl;
  proxyFile.oss << "  end" << endl;
  proxyFile.oss << "  methods" << endl;

  // Constructor
  proxyFile.oss << "    function obj = " << matlabName << "(varargin)" << endl;
  // Special pointer constructors - one in MATLAB to create an object and
	// assign a pointer returned from a C++ function.  In turn this MATLAB
	// constructor calls a special C++ function that just adds the object to
	// its collector.  This allows wrapped functions to return objects in
	// other wrap modules - to add these to their collectors the pointer is
	// passed from one C++ module into matlab then back into the other C++
	// module.
	{
		int id = functionNames.size();
		const string functionName = pointer_constructor_fragments(proxyFile, wrapperFile, wrapperName, id);
		functionNames.push_back(functionName);
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
	proxyFile.oss << "        error('Arguments do not match any overload of " << matlabName << " constructor');" << endl;
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
	BOOST_FOREACH(const Methods::value_type& name_m, methods) {
		const Method& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabName, wrapperName, using_namespaces, functionNames);
		proxyFile.oss << "\n";
		wrapperFile.oss << "\n";
	}

	proxyFile.oss << "  end\n";
	proxyFile.oss << "\n";
	proxyFile.oss << "  methods(Static = true)\n";

	// Static methods
	BOOST_FOREACH(const StaticMethods::value_type& name_m, static_methods) {
		const StaticMethod& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabName, wrapperName, using_namespaces, functionNames);
		proxyFile.oss << "\n";
		wrapperFile.oss << "\n";
	}

	proxyFile.oss << "  end" << endl;
	proxyFile.oss << "end" << endl;

  // Close file
  proxyFile.emit(true);
}

/* ************************************************************************* */
string Class::qualifiedName(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces)
		result += ns + delim;
	return result + name;
}

/* ************************************************************************* */
string Class::pointer_constructor_fragments(FileWriter& proxyFile, FileWriter& wrapperFile, const string& wrapperName, int id) const {
	
	static const uint64_t ptr_constructor_key =
		(uint64_t('G') << 56) |
		(uint64_t('T') << 48) |
		(uint64_t('S') << 40) |
		(uint64_t('A') << 32) |
		(uint64_t('M') << 24) |
		(uint64_t('p') << 16) |
		(uint64_t('t') << 8) |
		(uint64_t('r'));

  const string matlabName = qualifiedName(), cppName = qualifiedName("::");
	const string wrapFunctionName = matlabName + "_constructor_" + boost::lexical_cast<string>(id);

	// MATLAB constructor that assigns pointer to matlab object then calls c++
	// function to add the object to the collector.
	proxyFile.oss << "      if nargin == 2 && isa(varargin{1}, 'uint64') && ";
	proxyFile.oss << "varargin{1} == uint64(" << ptr_constructor_key << ")\n";
	proxyFile.oss << "        obj.self = varargin{2};\n";
	proxyFile.oss << "        " << wrapperName << "(obj.self);\n";

	// C++ function to add pointer from MATLAB to collector.  The pointer always
	// comes from a C++ return value; this mechanism allows the object to be added
	// to a collector in a different wrap module.
  wrapperFile.oss << "void " << wrapFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  wrapperFile.oss << "{\n";
	wrapperFile.oss << "  mexAtExit(&_deleteAllObjects);\n";
	generateUsingNamespace(wrapperFile, using_namespaces);
  // Typedef boost::shared_ptr
	wrapperFile.oss << "  typedef boost::shared_ptr<"  << cppName << "> Shared;\n";
	wrapperFile.oss << "\n";
	// Get self pointer passed in
	wrapperFile.oss << "  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));\n";
	// Add to collector
	wrapperFile.oss << "  collector_" << matlabName << ".insert(self);\n";
	wrapperFile.oss << "}\n";

	return wrapFunctionName;
}

/* ************************************************************************* */
