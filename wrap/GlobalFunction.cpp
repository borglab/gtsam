/**
 * @file GlobalFunction.cpp
 *
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#include "GlobalFunction.h"
#include "Class.h"
#include "utilities.h"

#include <boost/lexical_cast.hpp>

namespace wrap {

using namespace std;

/* ************************************************************************* */
void GlobalFunction::addOverload(const Qualified& overload,
    const ArgumentList& args, const ReturnValue& retVal, const std::string& _includeFile,
    boost::optional<const Qualified> instName, bool verbose) {
  FullyOverloadedFunction::addOverload(overload.name(), args, retVal, instName,
      verbose);
  overloads.push_back(overload);
  includeFile = _includeFile;
}

/* ************************************************************************* */
void GlobalFunction::matlab_proxy(const string& toolboxPath,
    const string& wrapperName, const TypeAttributesTable& typeAttributes,
    FileWriter& file, vector<string>& functionNames) const {

  // cluster overloads with same namespace
  // create new GlobalFunction structures around namespaces - same namespaces and names are overloads
  // map of namespace to global function
  typedef map<string, GlobalFunction> GlobalFunctionMap;
  GlobalFunctionMap grouped_functions;
  for (size_t i = 0; i < overloads.size(); ++i) {
    Qualified overload = overloads.at(i);
    // use concatenated namespaces as key
    string str_ns = qualifiedName("", overload.namespaces());
    const ReturnValue& ret = returnValue(i);
    const ArgumentList& args = argumentList(i);
    grouped_functions[str_ns].addOverload(overload, args, ret);
  }

  size_t lastcheck = grouped_functions.size();
  for(const GlobalFunctionMap::value_type& p: grouped_functions) {
    p.second.generateSingleFunction(toolboxPath, wrapperName, typeAttributes,
        file, functionNames);
    if (--lastcheck != 0)
      file.oss << endl;
  }
}

/* ************************************************************************* */
void GlobalFunction::generateSingleFunction(const string& toolboxPath,
    const string& wrapperName, const TypeAttributesTable& typeAttributes,
    FileWriter& file, vector<string>& functionNames) const {

  // create the folder for the namespace
  const Qualified& overload1 = overloads.front();
  createNamespaceStructure(overload1.namespaces(), toolboxPath);

  // open destination mfunctionFileName
  string mfunctionFileName = overload1.matlabName(toolboxPath);
  FileWriter mfunctionFile(mfunctionFileName, verbose_, "%");

  // get the name of actual matlab object
  const string matlabQualName = overload1.qualifiedName(".");
  const string matlabUniqueName = overload1.qualifiedName("");
  const string cppName = overload1.qualifiedName("::");

  mfunctionFile.oss << "function varargout = " << name_ << "(varargin)\n";

  for (size_t i = 0; i < nrOverloads(); ++i) {
    const ArgumentList& args = argumentList(i);
    const ReturnValue& returnVal = returnValue(i);

    const int id = functionNames.size();

    // Output proxy matlab code
    mfunctionFile.oss << "      " << (i == 0 ? "" : "else");
    emit_conditional_call(mfunctionFile, returnVal, args, wrapperName, id);

    // Output C++ wrapper code

    const string wrapFunctionName = matlabUniqueName + "_"
        + boost::lexical_cast<string>(id);

    // call
    file.oss << "void " << wrapFunctionName
        << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
    // start
    file.oss << "{\n";

    returnVal.wrapTypeUnwrap(file);

    // check arguments
    // NOTE: for static functions, there is no object passed
    file.oss << "  checkArguments(\"" << matlabUniqueName
        << "\",nargout,nargin," << args.size() << ");\n";

    // unwrap arguments, see Argument.cpp
    args.matlab_unwrap(file, 0); // We start at 0 because there is no self object

    // call method with default type and wrap result
    if (returnVal.type1.name() != "void")
      returnVal.wrap_result(cppName + "(" + args.names() + ")", file,
          typeAttributes);
    else
      file.oss << cppName + "(" + args.names() + ");\n";

    // finish
    file.oss << "}\n";

    // Add to function list
    functionNames.push_back(wrapFunctionName);
  }

  mfunctionFile.oss << "      else\n";
  mfunctionFile.oss
      << "        error('Arguments do not match any overload of function "
      << matlabQualName << "');" << endl;
  mfunctionFile.oss << "      end" << endl;

  // Close file
  mfunctionFile.emit(true);
}

/* ************************************************************************* */
void GlobalFunction::python_wrapper(FileWriter& wrapperFile) const {
  wrapperFile.oss << "def(\"" << name_ << "\", " << name_ << ");\n";
}

/* ************************************************************************* */
void GlobalFunction::emit_cython_pxd(FileWriter& file) const {
  file.oss << "cdef extern from \"" << includeFile << "\" namespace \""
                << overloads[0].qualifiedNamespaces("::") 
                << "\":" << endl;
  for (size_t i = 0; i < nrOverloads(); ++i) {
    file.oss << "        ";
    returnVals_[i].emit_cython_pxd(file, "", vector<string>());
    file.oss << pxdName() + " \"" + overloads[0].qualifiedName("::") +
                    "\"(";
    argumentList(i).emit_cython_pxd(file, "", vector<string>());
    file.oss << ")";
    file.oss << "\n";
  }
}

/* ************************************************************************* */
void GlobalFunction::emit_cython_pyx_no_overload(FileWriter& file) const {
  string funcName = pyxName();

  // Function definition
  file.oss << "def " << funcName;

  // modify name of function instantiation as python doesn't allow overloads
  // e.g. template<T={A,B,C}> funcName(...) --> funcNameA, funcNameB, funcNameC
  if (templateArgValue_) file.oss << templateArgValue_->pyxClassName();

  // funtion arguments
  file.oss << "(";
  argumentList(0).emit_cython_pyx(file);
  file.oss << "):\n";

  /// Call cython corresponding function and return
  file.oss << argumentList(0).pyx_convertEigenTypeAndStorageOrder("    ");
  string ret = pyx_functionCall("", pxdName(), 0);
  if (!returnVals_[0].isVoid()) {
    file.oss << "    cdef " << returnVals_[0].pyx_returnType()
             << " ret = " << ret << "\n";
    file.oss << "    return " << returnVals_[0].pyx_casting("ret") << "\n";
  } else {
    file.oss << "    " << ret << "\n";
  }
}

/* ************************************************************************* */
void GlobalFunction::emit_cython_pyx(FileWriter& file) const {
  string funcName = pyxName();

  size_t N = nrOverloads();
  if (N == 1) {
    emit_cython_pyx_no_overload(file);
    return;
  }

  // Dealing with overloads..
  file.oss << "def " << funcName << "(*args, **kwargs):\n";
  for (size_t i = 0; i < N; ++i) {
    file.oss << "    success, results = " << funcName << "_" << i
             << "(args, kwargs)\n";
    file.oss << "    if success:\n            return results\n";
  }
  file.oss << "    raise TypeError('Could not find the correct overload')\n";

  for (size_t i = 0; i < N; ++i) {
    ArgumentList args = argumentList(i);
    file.oss << "def " + funcName + "_" + to_string(i) + "(args, kwargs):\n";
    file.oss << "    cdef list __params\n";
    if (!returnVals_[i].isVoid()) {
      file.oss << "    cdef " << returnVals_[i].pyx_returnType() << " return_value\n";
    }
    file.oss << "    try:\n";
    file.oss << pyx_resolveOverloadParams(args, false, 2); // lazy: always return None even if it's a void function

    /// Call corresponding cython function
    file.oss << argumentList(i).pyx_convertEigenTypeAndStorageOrder("        ");
    string call = pyx_functionCall("", pxdName(), i);
    if (!returnVals_[i].isVoid()) {
      file.oss << "        return_value = " << call << "\n";
      file.oss << "        return True, " << returnVals_[i].pyx_casting("return_value") << "\n";
    } else {
      file.oss << "        " << call << "\n";
      file.oss << "        return True, None\n";
    }
    file.oss << "    except:\n";
    file.oss << "        return False, None\n\n";
  }
}
/* ************************************************************************* */

} // \namespace wrap

