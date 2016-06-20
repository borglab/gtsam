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

#include "Class.h"
#include "utilities.h"
#include "Argument.h"

#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <vector> 
#include <iostream> 
#include <fstream> 
#include <sstream>
#include <iterator>     // std::ostream_iterator
//#include <cstdint> // on Linux GCC: fails with error regarding needing C++0x std flags 
//#include <cinttypes> // same failure as above 
#include <stdint.h> // works on Linux GCC 
using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Class::assignParent(const Qualified& parent) {
  parentClass.reset(parent);
}

/* ************************************************************************* */
boost::optional<string> Class::qualifiedParent() const {
  boost::optional<string> result = boost::none;
  if (parentClass)
    result = parentClass->qualifiedName("::");
  return result;
}

/* ************************************************************************* */
static void handleException(const out_of_range& oor,
    const Class::Methods& methods) {
  cerr << "Class::method: key not found: " << oor.what() << ", methods are:\n";
  using boost::adaptors::map_keys;
  ostream_iterator<string> out_it(cerr, "\n");
  boost::copy(methods | map_keys, out_it);
}

/* ************************************************************************* */
Method& Class::mutableMethod(Str key) {
  try {
    return methods_.at(key);
  } catch (const out_of_range& oor) {
    handleException(oor, methods_);
    throw runtime_error("Internal error in wrap");
  }
}

/* ************************************************************************* */
const Method& Class::method(Str key) const {
  try {
    return methods_.at(key);
  } catch (const out_of_range& oor) {
    handleException(oor, methods_);
    throw runtime_error("Internal error in wrap");
  }
}

/* ************************************************************************* */
void Class::matlab_proxy(Str toolboxPath, Str wrapperName,
    const TypeAttributesTable& typeAttributes, FileWriter& wrapperFile,
    vector<string>& functionNames) const {

  // Create namespace folders 
  createNamespaceStructure(namespaces(), toolboxPath);

  // open destination classFile 
  string classFile = matlabName(toolboxPath);
  FileWriter proxyFile(classFile, verbose_, "%");

  // get the name of actual matlab object 
  const string matlabQualName = qualifiedName(".");
  const string matlabUniqueName = qualifiedName();
  const string cppName = qualifiedName("::");

  // emit class proxy code 
  // we want our class to inherit the handle class for memory purposes 
  const string parent =
      parentClass ? parentClass->qualifiedName(".") : "handle";
  comment_fragment(proxyFile);
  proxyFile.oss << "classdef " << name() << " < " << parent << endl;
  proxyFile.oss << "  properties\n";
  proxyFile.oss << "    ptr_" << matlabUniqueName << " = 0\n";
  proxyFile.oss << "  end\n";
  proxyFile.oss << "  methods\n";

  // Constructor 
  proxyFile.oss << "    function obj = " << name() << "(varargin)\n";
  // Special pointer constructors - one in MATLAB to create an object and 
  // assign a pointer returned from a C++ function.  In turn this MATLAB 
  // constructor calls a special C++ function that just adds the object to 
  // its collector.  This allows wrapped functions to return objects in 
  // other wrap modules - to add these to their collectors the pointer is 
  // passed from one C++ module into matlab then back into the other C++ 
  // module. 
  pointer_constructor_fragments(proxyFile, wrapperFile, wrapperName,
      functionNames);
  wrapperFile.oss << "\n";

  // Regular constructors 
  boost::optional<string> cppBaseName = qualifiedParent();
  for (size_t i = 0; i < constructor.nrOverloads(); i++) {
    ArgumentList args = constructor.argumentList(i);
    const int id = (int) functionNames.size();
    constructor.proxy_fragment(proxyFile, wrapperName, (bool) parentClass, id,
        args);
    const string wrapFunctionName = constructor.wrapper_fragment(wrapperFile,
        cppName, matlabUniqueName, cppBaseName, id, args);
    wrapperFile.oss << "\n";
    functionNames.push_back(wrapFunctionName);
  }
  proxyFile.oss << "      else\n";
  proxyFile.oss << "        error('Arguments do not match any overload of "
      << matlabQualName << " constructor');\n";
  proxyFile.oss << "      end\n";
  if (parentClass)
    proxyFile.oss << "      obj = obj@" << parentClass->qualifiedName(".")
        << "(uint64(" << ptr_constructor_key << "), base_ptr);\n";
  proxyFile.oss << "      obj.ptr_" << matlabUniqueName << " = my_ptr;\n";
  proxyFile.oss << "    end\n\n";

  // Deconstructor 
  {
    const int id = (int) functionNames.size();
    deconstructor.proxy_fragment(proxyFile, wrapperName, matlabUniqueName, id);
    proxyFile.oss << "\n";
    const string functionName = deconstructor.wrapper_fragment(wrapperFile,
        cppName, matlabUniqueName, id);
    wrapperFile.oss << "\n";
    functionNames.push_back(functionName);
  }
  proxyFile.oss
      << "    function display(obj), obj.print(''); end\n    %DISPLAY Calls print on the object\n";
  proxyFile.oss
      << "    function disp(obj), obj.display; end\n    %DISP Calls print on the object\n";

  // Methods 
  for(const Methods::value_type& name_m: methods_) {
    const Method& m = name_m.second;
    m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabQualName,
        matlabUniqueName, wrapperName, typeAttributes, functionNames);
    proxyFile.oss << "\n";
    wrapperFile.oss << "\n";
  }
  if (hasSerialization)
    serialization_fragments(proxyFile, wrapperFile, wrapperName, functionNames);

  proxyFile.oss << "  end\n";
  proxyFile.oss << "\n";
  proxyFile.oss << "  methods(Static = true)\n";

  // Static methods 
  for(const StaticMethods::value_type& name_m: static_methods) {
    const StaticMethod& m = name_m.second;
    m.proxy_wrapper_fragments(proxyFile, wrapperFile, cppName, matlabQualName,
        matlabUniqueName, wrapperName, typeAttributes, functionNames);
    proxyFile.oss << "\n";
    wrapperFile.oss << "\n";
  }
  if (hasSerialization)
    deserialization_fragments(proxyFile, wrapperFile, wrapperName,
        functionNames);

  proxyFile.oss << "  end\n";
  proxyFile.oss << "end\n";

  // Close file 
  proxyFile.emit(true);
}

/* ************************************************************************* */
void Class::pointer_constructor_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, Str wrapperName,
    vector<string>& functionNames) const {

  const string matlabUniqueName = qualifiedName();
  const string cppName = qualifiedName("::");

  const int collectorInsertId = (int) functionNames.size();
  const string collectorInsertFunctionName = matlabUniqueName
      + "_collectorInsertAndMakeBase_"
      + boost::lexical_cast<string>(collectorInsertId);
  functionNames.push_back(collectorInsertFunctionName);

  int upcastFromVoidId;
  string upcastFromVoidFunctionName;
  if (isVirtual) {
    upcastFromVoidId = (int) functionNames.size();
    upcastFromVoidFunctionName = matlabUniqueName + "_upcastFromVoid_"
        + boost::lexical_cast<string>(upcastFromVoidId);
    functionNames.push_back(upcastFromVoidFunctionName);
  }

  // MATLAB constructor that assigns pointer to matlab object then calls c++ 
  // function to add the object to the collector. 
  if (isVirtual) {
    proxyFile.oss
        << "      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void')))";
  } else {
    proxyFile.oss << "      if nargin == 2";
  }
  proxyFile.oss << " && isa(varargin{1}, 'uint64') && varargin{1} == uint64("
      << ptr_constructor_key << ")\n";
  if (isVirtual) {
    proxyFile.oss << "        if nargin == 2\n";
    proxyFile.oss << "          my_ptr = varargin{2};\n";
    proxyFile.oss << "        else\n";
    proxyFile.oss << "          my_ptr = " << wrapperName << "("
        << upcastFromVoidId << ", varargin{2});\n";
    proxyFile.oss << "        end\n";
  } else {
    proxyFile.oss << "        my_ptr = varargin{2};\n";
  }
  if (!parentClass) // If this class has a base class, we'll get a base class pointer back
    proxyFile.oss << "        ";
  else
    proxyFile.oss << "        base_ptr = ";
  proxyFile.oss << wrapperName << "(" << collectorInsertId << ", my_ptr);\n"; // Call collector insert and get base class ptr 

  // C++ function to add pointer from MATLAB to collector.  The pointer always 
  // comes from a C++ return value; this mechanism allows the object to be added 
  // to a collector in a different wrap module.  If this class has a base class, 
  // a new pointer to the base class is allocated and returned. 
  wrapperFile.oss << "void " << collectorInsertFunctionName
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  wrapperFile.oss << "{\n";
  wrapperFile.oss << "  mexAtExit(&_deleteAllObjects);\n";
  // Typedef boost::shared_ptr 
  wrapperFile.oss << "  typedef boost::shared_ptr<" << cppName << "> Shared;\n";
  wrapperFile.oss << "\n";
  // Get self pointer passed in 
  wrapperFile.oss
      << "  Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));\n";
  // Add to collector 
  wrapperFile.oss << "  collector_" << matlabUniqueName << ".insert(self);\n";
  // If we have a base class, return the base class pointer (MATLAB will call the base class collectorInsertAndMakeBase to add this to the collector and recurse the heirarchy) 
  boost::optional<string> cppBaseName = qualifiedParent();
  if (cppBaseName) {
    wrapperFile.oss << "\n";
    wrapperFile.oss << "  typedef boost::shared_ptr<" << *cppBaseName
        << "> SharedBase;\n";
    wrapperFile.oss
        << "  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);\n";
    wrapperFile.oss
        << "  *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);\n";
  }
  wrapperFile.oss << "}\n";

  // If this is a virtual function, C++ function to dynamic upcast it from a 
  // shared_ptr<void>.  This mechanism allows automatic dynamic creation of the 
  // real underlying derived-most class when a C++ method returns a virtual 
  // base class. 
  if (isVirtual)
    wrapperFile.oss << "\n"
        "void " << upcastFromVoidFunctionName
        << "(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {\n"
            "  mexAtExit(&_deleteAllObjects);\n"
            "  typedef boost::shared_ptr<" << cppName
        << "> Shared;\n"
            "  boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));\n"
            "  out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);\n"
            "  Shared *self = new Shared(boost::static_pointer_cast<" << cppName
        << ">(*asVoid));\n"
            "  *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;\n"
            "}\n";
}

/* ************************************************************************* */
Class Class::expandTemplate(const TemplateSubstitution& ts) const {
  Class inst = *this;
  inst.methods_ = expandMethodTemplate(methods_, ts);
  inst.static_methods = expandMethodTemplate(static_methods, ts);
  inst.constructor = constructor.expandTemplate(ts);
  inst.deconstructor.name = inst.name();
  return inst;
}

/* ************************************************************************* */
vector<Class> Class::expandTemplate(Str templateArg,
    const vector<Qualified>& instantiations) const {
  vector<Class> result;
  for(const Qualified& instName: instantiations) {
    Qualified expandedClass = (Qualified) (*this);
    expandedClass.expand(instName.name());
    const TemplateSubstitution ts(templateArg, instName, expandedClass);
    Class inst = expandTemplate(ts);
    inst.name_ = expandedClass.name();
    inst.templateArgs.clear();
    inst.typedefName = qualifiedName("::") + "<" + instName.qualifiedName("::")
        + ">";
    result.push_back(inst);
  }
  return result;
}

/* ************************************************************************* */
vector<Class> Class::expandTemplate(Str templateArg,
    const vector<int>& integers) const {
  vector<Class> result;
  for(int i: integers) {
    Qualified expandedClass = (Qualified) (*this);
    stringstream ss; ss << i;
    string instName = ss.str();
    expandedClass.expand(instName);
    const TemplateSubstitution ts(templateArg, instName, expandedClass);
    Class inst = expandTemplate(ts);
    inst.name_ = expandedClass.name();
    inst.templateArgs.clear();
    inst.typedefName = qualifiedName("::") + "<" + instName + ">";
    result.push_back(inst);
  }
  return result;
}

/* ************************************************************************* */
void Class::addMethod(bool verbose, bool is_const, Str methodName,
    const ArgumentList& argumentList, const ReturnValue& returnValue,
    const Template& tmplate) {
  // Check if templated
  if (tmplate.valid()) {
    // Create method to expand
    // For all values of the template argument, create a new method
    for(const Qualified& instName: tmplate.argValues()) {
      const TemplateSubstitution ts(tmplate.argName(), instName, *this);
      // substitute template in arguments
      ArgumentList expandedArgs = argumentList.expandTemplate(ts);
      // do the same for return type
      ReturnValue expandedRetVal = returnValue.expandTemplate(ts);
      // Now stick in new overload stack with expandedMethodName key
      // but note we use the same, unexpanded methodName in overload
      string expandedMethodName = methodName + instName.name();
      methods_[expandedMethodName].addOverload(methodName, expandedArgs,
          expandedRetVal, is_const, instName, verbose);
    }
  } else
    // just add overload
    methods_[methodName].addOverload(methodName, argumentList, returnValue,
        is_const, boost::none, verbose);
}

/* ************************************************************************* */
void Class::erase_serialization() {
  Methods::iterator it = methods_.find("serializable");
  if (it != methods_.end()) {
#ifndef WRAP_DISABLE_SERIALIZE
    isSerializable = true;
#else
    // cout << "Ignoring serializable() flag in class " << name << endl;
#endif
    methods_.erase(it);
  }

  it = methods_.find("serialize");
  if (it != methods_.end()) {
#ifndef WRAP_DISABLE_SERIALIZE
    isSerializable = true;
    hasSerialization = true;
#else
    // cout << "Ignoring serialize() flag in class " << name << endl;
#endif
    methods_.erase(it);
  }
}

/* ************************************************************************* */
void Class::verifyAll(vector<string>& validTypes, bool& hasSerialiable) const {

  hasSerialiable |= isSerializable;

  // verify all of the function arguments
  //TODO:verifyArguments<ArgumentList>(validTypes, constructor.args_list);
  verifyArguments<StaticMethod>(validTypes, static_methods);
  verifyArguments<Method>(validTypes, methods_);

  // verify function return types
  verifyReturnTypes<StaticMethod>(validTypes, static_methods);
  verifyReturnTypes<Method>(validTypes, methods_);

  // verify parents
  boost::optional<string> parent = qualifiedParent();
  if (parent
      && find(validTypes.begin(), validTypes.end(), *parent)
          == validTypes.end())
    throw DependencyMissing(*parent, qualifiedName("::"));
}

/* ************************************************************************* */
void Class::appendInheritedMethods(const Class& cls,
    const vector<Class>& classes) {

  if (cls.parentClass) {

    // Find parent
    for(const Class& parent: classes) {
      // We found a parent class for our parent, TODO improve !
      if (parent.name() == cls.parentClass->name()) {
        methods_.insert(parent.methods_.begin(), parent.methods_.end());
        appendInheritedMethods(parent, classes);
      }
    }
  }
}

/* ************************************************************************* */
string Class::getTypedef() const {
  string result;
  for(Str namesp: namespaces()) {
    result += ("namespace " + namesp + " { ");
  }
  result += ("typedef " + typedefName + " " + name() + ";");
  for (size_t i = 0; i < namespaces().size(); ++i) {
    result += " }";
  }
  return result;
}

/* ************************************************************************* */
void Class::comment_fragment(FileWriter& proxyFile) const {
  proxyFile.oss << "%class " << name() << ", see Doxygen page for details\n";
  proxyFile.oss
      << "%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html\n";

  constructor.comment_fragment(proxyFile);

  if (!methods_.empty())
    proxyFile.oss << "%\n%-------Methods-------\n";
  for(const Methods::value_type& name_m: methods_)
    name_m.second.comment_fragment(proxyFile);

  if (!static_methods.empty())
    proxyFile.oss << "%\n%-------Static Methods-------\n";
  for(const StaticMethods::value_type& name_m: static_methods)
    name_m.second.comment_fragment(proxyFile);

  if (hasSerialization) {
    proxyFile.oss << "%\n%-------Serialization Interface-------\n";
    proxyFile.oss << "%string_serialize() : returns string\n";
    proxyFile.oss << "%string_deserialize(string serialized) : returns "
        << name() << "\n";
  }

  proxyFile.oss << "%\n";
}

/* ************************************************************************* */
void Class::serialization_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, Str wrapperName,
    vector<string>& functionNames) const {

//void Point3_string_serialize_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
//{
//  typedef boost::shared_ptr<Point3> Shared;
//  checkArguments("string_serialize",nargout,nargin-1,0);
//  Shared obj = unwrap_shared_ptr<Point3>(in[0], "ptr_Point3");
//  ostringstream out_archive_stream;
//  boost::archive::text_oarchive out_archive(out_archive_stream);
//  out_archive << *obj;
//  out[0] = wrap< string >(out_archive_stream.str());
//}

  int serialize_id = functionNames.size();
  const string matlabQualName = qualifiedName(".");
  const string matlabUniqueName = qualifiedName();
  const string cppClassName = qualifiedName("::");
  const string wrapFunctionNameSerialize = matlabUniqueName
      + "_string_serialize_" + boost::lexical_cast<string>(serialize_id);
  functionNames.push_back(wrapFunctionNameSerialize);

  // call
  //void Point3_string_serialize_17(int nargout, mxArray *out[], int nargin, const mxArray *in[])
  wrapperFile.oss << "void " << wrapFunctionNameSerialize
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  wrapperFile.oss << "{\n";
  wrapperFile.oss << "  typedef boost::shared_ptr<" << cppClassName
      << "> Shared;" << endl;

  // check arguments - for serialize, no arguments
  // example: checkArguments("string_serialize",nargout,nargin-1,0);
  wrapperFile.oss
      << "  checkArguments(\"string_serialize\",nargout,nargin-1,0);\n";

  // get class pointer
  // example: Shared obj = unwrap_shared_ptr<Point3>(in[0], "ptr_Point3");
  wrapperFile.oss << "  Shared obj = unwrap_shared_ptr<" << cppClassName
      << ">(in[0], \"ptr_" << matlabUniqueName << "\");" << endl;

  // Serialization boilerplate
  wrapperFile.oss << "  ostringstream out_archive_stream;\n";
  wrapperFile.oss
      << "  boost::archive::text_oarchive out_archive(out_archive_stream);\n";
  wrapperFile.oss << "  out_archive << *obj;\n";
  wrapperFile.oss << "  out[0] = wrap< string >(out_archive_stream.str());\n";

  // finish
  wrapperFile.oss << "}\n";

  // Generate code for matlab function
//  function varargout string_serialize(this, varargin)
//  % STRING_SERIALIZE usage: string_serialize() : returns string
//  % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
//    if length(varargin) == 0
//      varargout{1} = geometry_wrapper(15, this, varargin{:});
//    else
//      error('Arguments do not match any overload of function Point3.string_serialize');
//    end
//  end

  proxyFile.oss
      << "    function varargout = string_serialize(this, varargin)\n";
  proxyFile.oss
      << "      % STRING_SERIALIZE usage: string_serialize() : returns string\n";
  proxyFile.oss
      << "      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html\n";
  proxyFile.oss << "      if length(varargin) == 0\n";
  proxyFile.oss << "        varargout{1} = " << wrapperName << "("
      << boost::lexical_cast<string>(serialize_id) << ", this, varargin{:});\n";
  proxyFile.oss << "      else\n";
  proxyFile.oss
      << "        error('Arguments do not match any overload of function "
      << matlabQualName << ".string_serialize');\n";
  proxyFile.oss << "      end\n";
  proxyFile.oss << "    end\n\n";

  // Generate code for matlab save function
//  function sobj = saveobj(obj)
//    % SAVEOBJ Saves the object to a matlab-readable format
//    sobj = obj.string_serialize();
//  end

  proxyFile.oss << "    function sobj = saveobj(obj)\n";
  proxyFile.oss
      << "      % SAVEOBJ Saves the object to a matlab-readable format\n";
  proxyFile.oss << "      sobj = obj.string_serialize();\n";
  proxyFile.oss << "    end\n";
}

/* ************************************************************************* */
void Class::deserialization_fragments(FileWriter& proxyFile,
    FileWriter& wrapperFile, Str wrapperName,
    vector<string>& functionNames) const {
  //void Point3_string_deserialize_18(int nargout, mxArray *out[], int nargin, const mxArray *in[])
  //{
  //  typedef boost::shared_ptr<Point3> Shared;
  //  checkArguments("Point3.string_deserialize",nargout,nargin,1);
  //  string serialized = unwrap< string >(in[0]);
  //  istringstream in_archive_stream(serialized);
  //  boost::archive::text_iarchive in_archive(in_archive_stream);
  //  Shared output(new Point3(0,0,0));
  //  in_archive >> *output;
  //  out[0] = wrap_shared_ptr(output,"Point3", false);
  //}
  int deserialize_id = functionNames.size();
  const string matlabQualName = qualifiedName(".");
  const string matlabUniqueName = qualifiedName();
  const string cppClassName = qualifiedName("::");
  const string wrapFunctionNameDeserialize = matlabUniqueName
      + "_string_deserialize_" + boost::lexical_cast<string>(deserialize_id);
  functionNames.push_back(wrapFunctionNameDeserialize);

  // call
  wrapperFile.oss << "void " << wrapFunctionNameDeserialize
      << "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
  wrapperFile.oss << "{\n";
  wrapperFile.oss << "  typedef boost::shared_ptr<" << cppClassName
      << "> Shared;" << endl;

  // check arguments - for deserialize, 1 string argument
  wrapperFile.oss << "  checkArguments(\"" << matlabUniqueName
      << ".string_deserialize\",nargout,nargin,1);\n";

  // string argument with deserialization boilerplate
  wrapperFile.oss << "  string serialized = unwrap< string >(in[0]);\n";
  wrapperFile.oss << "  istringstream in_archive_stream(serialized);\n";
  wrapperFile.oss
      << "  boost::archive::text_iarchive in_archive(in_archive_stream);\n";
  wrapperFile.oss << "  Shared output(new " << cppClassName << "());\n";
  wrapperFile.oss << "  in_archive >> *output;\n";
  wrapperFile.oss << "  out[0] = wrap_shared_ptr(output,\"" << matlabQualName
      << "\", false);\n";
  wrapperFile.oss << "}\n";

  // Generate matlab function
//    function varargout = string_deserialize(varargin)
//    % STRING_DESERIALIZE usage: string_deserialize() : returns Point3
//    % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
//      if length(varargin) == 1
//        varargout{1} = geometry_wrapper(18, varargin{:});
//      else
//        error('Arguments do not match any overload of function Point3.string_deserialize');
//      end
//    end

  proxyFile.oss << "    function varargout = string_deserialize(varargin)\n";
  proxyFile.oss
      << "      % STRING_DESERIALIZE usage: string_deserialize() : returns "
      << matlabQualName << "\n";
  proxyFile.oss
      << "      % Doxygen can be found at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html\n";
  proxyFile.oss << "      if length(varargin) == 1\n";
  proxyFile.oss << "        varargout{1} = " << wrapperName << "("
      << boost::lexical_cast<string>(deserialize_id) << ", varargin{:});\n";
  proxyFile.oss << "      else\n";
  proxyFile.oss
      << "        error('Arguments do not match any overload of function "
      << matlabQualName << ".string_deserialize');\n";
  proxyFile.oss << "      end\n";
  proxyFile.oss << "    end\n\n";

  // Generate matlab load function
//    function obj = loadobj(sobj)
//      % LOADOBJ Saves the object to a matlab-readable format
//      obj = Point3.string_deserialize(sobj);
//    end

  proxyFile.oss << "    function obj = loadobj(sobj)\n";
  proxyFile.oss
      << "      % LOADOBJ Saves the object to a matlab-readable format\n";
  proxyFile.oss << "      obj = " << matlabQualName
      << ".string_deserialize(sobj);\n";
  proxyFile.oss << "    end" << endl;
}

/* ************************************************************************* */
string Class::getSerializationExport() const {
  //BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsamSharedDiagonal");
  return "BOOST_CLASS_EXPORT_GUID(" + qualifiedName("::") + ", \""
      + qualifiedName() + "\");";
}

/* ************************************************************************* */
void Class::python_wrapper(FileWriter& wrapperFile) const {
  wrapperFile.oss << "class_<" << name() << ">(\"" << name() << "\")\n";
  constructor.python_wrapper(wrapperFile, name());
  for(const StaticMethod& m: static_methods | boost::adaptors::map_values)
    m.python_wrapper(wrapperFile, name());
  for(const Method& m: methods_ | boost::adaptors::map_values)
    m.python_wrapper(wrapperFile, name());
  wrapperFile.oss << ";\n\n";
}

/* ************************************************************************* */
