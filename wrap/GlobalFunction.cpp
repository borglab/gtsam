/**
 * @file GlobalFunction.cpp
 *
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#include "GlobalFunction.h"
#include "utilities.h"

#include <boost/lexical_cast.hpp>

namespace wrap {

using namespace std;

/* ************************************************************************* */
void GlobalFunction::addOverload(const Qualified& overload,
    const ArgumentList& args, const ReturnValue& retVal,
    boost::optional<const Qualified> instName, bool verbose) {
  FullyOverloadedFunction::addOverload(overload.name(), args, retVal, instName,
      verbose);
  overloads.push_back(overload);
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

} // \namespace wrap

