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
//#include <cstdint> // on Linux GCC: fails with error regarding needing C++0x std flags
//#include <cinttypes> // same failure as above
#include <stdint.h> // works on Linux GCC

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "Class.h"
#include "utilities.h"
#include "Argument.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Class::matlab_proxy(const string& classFile, const string& wrapperName,
												 const ReturnValue::TypeAttributesTable& typeAttributes,
												 FileWriter& wrapperFile, vector<string>& functionNames) const {
  // open destination classFile
  FileWriter proxyFile(classFile, verbose_, "%");

  // get the name of actual matlab object
  const string matlabName = qualifiedName(), cppName = qualifiedName("::");
	const string matlabBaseName = wrap::qualifiedName("", qualifiedParent);
	const string cppBaseName = wrap::qualifiedName("::", qualifiedParent);

  // emit class proxy code
  // we want our class to inherit the handle class for memory purposes
	const string parent = qualifiedParent.empty() ?
		"handle" : ::wrap::qualifiedName("", qualifiedParent);
  proxyFile.oss << "classdef " << matlabName << " < " << parent << endl;
  proxyFile.oss << "  properties" << endl;
  proxyFile.oss << "    ptr_" << matlabName << " = 0" << endl;
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
	pointer_constructor_fragments(proxyFile, wrapperFile, wrapperName, functionNames);
	wrapperFile.oss << "\n";
	// Regular constructors
  BOOST_FOREACH(ArgumentList a, constructor.args_list)
  {
		const int id = functionNames.size();
		constructor.proxy_fragment(proxyFile, wrapperName, matlabName, matlabBaseName, id, a);
		const string wrapFunctionName = constructor.wrapper_fragment(wrapperFile,
			cppName, matlabName, cppBaseName, id, using_namespaces, a);
		wrapperFile.oss << "\n";
    functionNames.push_back(wrapFunctionName);
  }
  proxyFile.oss << "      else\n";
	proxyFile.oss << "        error('Arguments do not match any overload of " << matlabName << " constructor');" << endl;
	proxyFile.oss << "      end\n";
	if(!qualifiedParent.empty())
		proxyFile.oss << "      obj = obj@" << matlabBaseName << "(uint64(" << ptr_constructor_key << "), base_ptr);\n";
	proxyFile.oss << "      obj.ptr_" << matlabName << " = my_ptr;\n";
  proxyFile.oss << "    end\n\n";

	// Deconstructor
	{
		const int id = functionNames.size();
		deconstructor.proxy_fragment(proxyFile, wrapperName, matlabName, id);
		proxyFile.oss << "\n";
		const string functionName = deconstructor.wrapper_fragment(wrapperFile, cppName, matlabName, id, using_namespaces);
		wrapperFile.oss << "\n";
		functionNames.push_back(functionName);
	}
  proxyFile.oss << "    function display(obj), obj.print(''); end\n\n";
  proxyFile.oss << "    function disp(obj), obj.display; end\n\n";

	// Methods
	BOOST_FOREACH(const Methods::value_type& name_m, methods) {
		const Method& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabName, wrapperName, using_namespaces, typeAttributes, functionNames);
		proxyFile.oss << "\n";
		wrapperFile.oss << "\n";
	}

	proxyFile.oss << "  end\n";
	proxyFile.oss << "\n";
	proxyFile.oss << "  methods(Static = true)\n";

	// Static methods
	BOOST_FOREACH(const StaticMethods::value_type& name_m, static_methods) {
		const StaticMethod& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabName, wrapperName, using_namespaces, typeAttributes, functionNames);
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
	return ::wrap::qualifiedName(delim, namespaces, name);
}

/* ************************************************************************* */
void Class::pointer_constructor_fragments(FileWriter& proxyFile, FileWriter& wrapperFile, const string& wrapperName, vector<string>& functionNames) const {

  const string matlabName = qualifiedName(), cppName = qualifiedName("::");
	const string baseMatlabName = wrap::qualifiedName("", qualifiedParent);
	const string baseCppName = wrap::qualifiedName("::", qualifiedParent);

	const int collectorInsertId = functionNames.size();
	const string collectorInsertFunctionName = matlabName + "_collectorInsertAndMakeBase_" + boost::lexical_cast<string>(collectorInsertId);
	functionNames.push_back(collectorInsertFunctionName);

	int upcastFromVoidId;
	string upcastFromVoidFunctionName;
	if(isVirtual) {
		upcastFromVoidId = functionNames.size();
		upcastFromVoidFunctionName = matlabName + "_upcastFromVoid_" + boost::lexical_cast<string>(upcastFromVoidId);
		functionNames.push_back(upcastFromVoidFunctionName);
	}

	// MATLAB constructor that assigns pointer to matlab object then calls c++
	// function to add the object to the collector.
	if(isVirtual) {
		proxyFile.oss << "      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void')))";
	} else {
		proxyFile.oss << "      if nargin == 2";
	}
	proxyFile.oss << " && isa(varargin{1}, 'uint64') && varargin{1} == uint64(" << ptr_constructor_key << ")\n";
	if(isVirtual) {
		proxyFile.oss << "        if nargin == 2\n";
		proxyFile.oss << "          my_ptr = varargin{2};\n";
		proxyFile.oss << "        else\n";
		proxyFile.oss << "          my_ptr = " << wrapperName << "(" << upcastFromVoidId << ", varargin{2});\n";
		proxyFile.oss << "        end\n";
	} else {
		proxyFile.oss << "        my_ptr = varargin{2};\n";
	}
	if(qualifiedParent.empty()) // If this class has a base class, we'll get a base class pointer back
		proxyFile.oss << "        ";
	else
		proxyFile.oss << "        base_ptr = ";
	proxyFile.oss << wrapperName << "(" << collectorInsertId << ", my_ptr);\n"; // Call collector insert and get base class ptr

	// C++ function to add pointer from MATLAB to collector.  The pointer always
	// comes from a C++ return value; this mechanism allows the object to be added
	// to a collector in a different wrap module.  If this class has a base class,
	// a new pointer to the base class is allocated and returned.
  wrapperFile.oss << "void " << collectorInsertFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])" << endl;
  wrapperFile.oss << "{\n";
	wrapperFile.oss << "  mexAtExit(&_deleteAllObjects);\n";
	generateUsingNamespace(wrapperFile, using_namespaces);
  // Typedef boost::shared_ptr
	wrapperFile.oss << "  typedef boost::shared_ptr<" << cppName << "> Shared;\n";
	wrapperFile.oss << "\n";
	// Get self pointer passed in
	wrapperFile.oss << "  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));\n";
	// Add to collector
	wrapperFile.oss << "  collector_" << matlabName << ".insert(self);\n";
	// If we have a base class, return the base class pointer (MATLAB will call the base class collectorInsertAndMakeBase to add this to the collector and recurse the heirarchy)
	if(!qualifiedParent.empty()) {
		wrapperFile.oss << "\n";
		wrapperFile.oss << "  typedef boost::shared_ptr<" << baseCppName << "> SharedBase;\n";
    wrapperFile.oss << "  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);\n";
		wrapperFile.oss << "  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);\n";
	}
	wrapperFile.oss << "}\n";

	// If this is a virtual function, C++ function to dynamic upcast it from a
	// shared_ptr<void>.  This mechanism allows automatic dynamic creation of the
	// real underlying derived-most class when a C++ method returns a virtual
	// base class.
	if(isVirtual)
		wrapperFile.oss <<
		"\n"
		"void " << upcastFromVoidFunctionName << "(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {\n"
		"  mexAtExit(&_deleteAllObjects);\n"
		"  typedef boost::shared_ptr<" << cppName << "> Shared;\n"
		"  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));\n"
		"  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);\n"
		"  Shared *self = new Shared(boost::static_pointer_cast<" << cppName << ">(*asVoid));\n"
		"  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;\n"
		"}\n";
}

/* ************************************************************************* */
