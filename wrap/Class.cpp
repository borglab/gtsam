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
 * @author Richard Roberts
 **/

#include <vector>
#include <iostream>
#include <fstream>
//#include <cstdint> // on Linux GCC: fails with error regarding needing C++0x std flags
//#include <cinttypes> // same failure as above
#include <stdint.h> // works on Linux GCC

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "Class.h"
#include "utilities.h"
#include "Argument.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Class::matlab_proxy(const string& toolboxPath, const string& wrapperName,
												 const TypeAttributesTable& typeAttributes,
												 FileWriter& wrapperFile, vector<string>& functionNames) const {
  
	// Create namespace folders
	{
		using namespace boost::filesystem;
		path curPath = toolboxPath;
		BOOST_FOREACH(const string& subdir, namespaces) {
			curPath /= "+" + subdir;
			if(!is_directory(curPath))
				if(exists("+" + subdir))
					throw OutputError("Need to write files to directory " + curPath.string() + ", which already exists as a file but is not a directory");
				else
					boost::filesystem::create_directory(curPath);
		}
	}

	// open destination classFile
	string classFile = toolboxPath;
	if(!namespaces.empty())
		classFile += "/+" + wrap::qualifiedName("/+", namespaces);
	classFile += "/" + name + ".m";
  FileWriter proxyFile(classFile, verbose_, "%");

  // get the name of actual matlab object
  const string matlabQualName = qualifiedName("."), matlabUniqueName = qualifiedName(), cppName = qualifiedName("::");
	const string matlabBaseName = wrap::qualifiedName(".", qualifiedParent);
	const string cppBaseName = wrap::qualifiedName("::", qualifiedParent);

  // emit class proxy code
  // we want our class to inherit the handle class for memory purposes
	const string parent = qualifiedParent.empty() ? "handle" : matlabBaseName;
  proxyFile.oss << "classdef " << name << " < " << parent << endl;
  proxyFile.oss << "  properties" << endl;
  proxyFile.oss << "    ptr_" << matlabUniqueName << " = 0" << endl;
  proxyFile.oss << "  end" << endl;
  proxyFile.oss << "  methods" << endl;

  // Constructor
  proxyFile.oss << "    function obj = " << name << "(varargin)" << endl;
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
		const int id = (int)functionNames.size();
		constructor.proxy_fragment(proxyFile, wrapperName, !qualifiedParent.empty(), id, a);
		const string wrapFunctionName = constructor.wrapper_fragment(wrapperFile,
			cppName, matlabUniqueName, cppBaseName, id, using_namespaces, a);
		wrapperFile.oss << "\n";
    functionNames.push_back(wrapFunctionName);
  }
  proxyFile.oss << "      else\n";
	proxyFile.oss << "        error('Arguments do not match any overload of " << matlabQualName << " constructor');" << endl;
	proxyFile.oss << "      end\n";
	if(!qualifiedParent.empty())
		proxyFile.oss << "      obj = obj@" << matlabBaseName << "(uint64(" << ptr_constructor_key << "), base_ptr);\n";
	proxyFile.oss << "      obj.ptr_" << matlabUniqueName << " = my_ptr;\n";
  proxyFile.oss << "    end\n\n";

	// Deconstructor
	{
		const int id = (int)functionNames.size();
		deconstructor.proxy_fragment(proxyFile, wrapperName, matlabUniqueName, id);
		proxyFile.oss << "\n";
		const string functionName = deconstructor.wrapper_fragment(wrapperFile, cppName, matlabUniqueName, id, using_namespaces);
		wrapperFile.oss << "\n";
		functionNames.push_back(functionName);
	}
  proxyFile.oss << "    function display(obj), obj.print(''); end\n\n";
  proxyFile.oss << "    function disp(obj), obj.display; end\n\n";

	// Methods
	BOOST_FOREACH(const Methods::value_type& name_m, methods) {
		const Method& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabQualName, matlabUniqueName, wrapperName, using_namespaces, typeAttributes, functionNames);
		proxyFile.oss << "\n";
		wrapperFile.oss << "\n";
	}

	proxyFile.oss << "  end\n";
	proxyFile.oss << "\n";
	proxyFile.oss << "  methods(Static = true)\n";

	// Static methods
	BOOST_FOREACH(const StaticMethods::value_type& name_m, static_methods) {
		const StaticMethod& m = name_m.second;
		m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabQualName, matlabUniqueName, wrapperName, using_namespaces, typeAttributes, functionNames);
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

  const string matlabUniqueName = qualifiedName(), cppName = qualifiedName("::");
	const string baseCppName = wrap::qualifiedName("::", qualifiedParent);

	const int collectorInsertId = (int)functionNames.size();
	const string collectorInsertFunctionName = matlabUniqueName + "_collectorInsertAndMakeBase_" + boost::lexical_cast<string>(collectorInsertId);
	functionNames.push_back(collectorInsertFunctionName);

	int upcastFromVoidId;
	string upcastFromVoidFunctionName;
	if(isVirtual) {
		upcastFromVoidId = (int)functionNames.size();
		upcastFromVoidFunctionName = matlabUniqueName + "_upcastFromVoid_" + boost::lexical_cast<string>(upcastFromVoidId);
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
	wrapperFile.oss << "  collector_" << matlabUniqueName << ".insert(self);\n";
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
vector<ArgumentList> expandArgumentListsTemplate(const vector<ArgumentList>& argLists, const string& templateArg, const vector<string>& instName) {
	vector<ArgumentList> result;
	BOOST_FOREACH(const ArgumentList& argList, argLists) {
		ArgumentList instArgList;
		BOOST_FOREACH(const Argument& arg, argList) {
			Argument instArg = arg;
			if(arg.type == templateArg) {
				instArg.namespaces.assign(instName.begin(), instName.end()-1);
				instArg.type = instName.back();
			}
			instArgList.push_back(instArg);
		}
		result.push_back(instArgList);
	}
	return result;
}

/* ************************************************************************* */
template<class METHOD>
map<string, METHOD> expandMethodTemplate(const map<string, METHOD>& methods, const string& templateArg, const vector<string>& instName) {
	map<string, METHOD> result;
	typedef pair<const string, METHOD> Name_Method;
	BOOST_FOREACH(const Name_Method& name_method, methods) {
		const METHOD& method = name_method.second;
		METHOD instMethod = method;
		instMethod.argLists = expandArgumentListsTemplate(method.argLists, templateArg, instName);
		instMethod.returnVals.clear();
		BOOST_FOREACH(const ReturnValue& retVal, method.returnVals) {
			ReturnValue instRetVal = retVal;
			if(retVal.type1 == templateArg) {
				instRetVal.namespaces1.assign(instName.begin(), instName.end()-1);
				instRetVal.type1 = instName.back();
			}
			if(retVal.type2 == templateArg) {
				instRetVal.namespaces2.assign(instName.begin(), instName.end()-1);
				instRetVal.type2 = instName.back();
			}
			instMethod.returnVals.push_back(instRetVal);
		}
		result.insert(make_pair(name_method.first, instMethod));
	}
	return result;
}

/* ************************************************************************* */
Class expandClassTemplate(const Class& cls, const string& templateArg, const vector<string>& instName) {
	Class inst;
	inst.name = cls.name;
	inst.templateArgs = cls.templateArgs;
	inst.typedefName = cls.typedefName;
	inst.isVirtual = cls.isVirtual;
	inst.qualifiedParent = cls.qualifiedParent;
	inst.methods = expandMethodTemplate(cls.methods, templateArg, instName);
	inst.static_methods = expandMethodTemplate(cls.static_methods, templateArg, instName);
	inst.namespaces = cls.namespaces;
	inst.using_namespaces = cls.using_namespaces;
	inst.constructor = cls.constructor;
	inst.constructor.args_list = expandArgumentListsTemplate(cls.constructor.args_list, templateArg, instName);
	inst.constructor.name = inst.name;
	inst.deconstructor = cls.deconstructor;
	inst.deconstructor.name = inst.name;
	inst.verbose_ = cls.verbose_;
	return inst;
}

/* ************************************************************************* */
vector<Class> Class::expandTemplate(const string& templateArg, const vector<vector<string> >& instantiations) const {
	vector<Class> result;
	BOOST_FOREACH(const vector<string>& instName, instantiations) {
		Class inst = expandClassTemplate(*this, templateArg, instName);
		inst.name = name + instName.back();
		inst.templateArgs.clear();
		inst.typedefName = qualifiedName("::") + "<" + wrap::qualifiedName("::", instName) + ">";
		result.push_back(inst);
	}
	return result;
}

/* ************************************************************************* */
Class Class::expandTemplate(const string& templateArg, const vector<string>& instantiation) const {
	return expandClassTemplate(*this, templateArg, instantiation);
}

/* ************************************************************************* */
std::string Class::getTypedef() const {
	string result;
	BOOST_FOREACH(const string& namesp, namespaces) {
		result += ("namespace " + namesp + " { ");
	}
	result += ("typedef " + typedefName + " " + name + ";");
	BOOST_FOREACH(const string& namesp, namespaces) {
		result += " }";
	}
	return result;
}

/* ************************************************************************* */
