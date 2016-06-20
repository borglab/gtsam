/* ---------------------------------------------------------------------------- 
 
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,  
 * Atlanta, Georgia 30332-0415 
 * All Rights Reserved 
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list) 
 
 * See LICENSE for the license information 
 
 * -------------------------------------------------------------------------- */ 
 
/** 
 * @file Module.ccp 
 * @author Frank Dellaert 
 * @author Alex Cunningham 
 * @author Andrew Melim 
 * @author Richard Roberts
 **/ 
 
#include "Module.h" 
#include "FileWriter.h" 
#include "TypeAttributesTable.h" 
#include "utilities.h"

#include <boost/filesystem.hpp> 
#include <boost/lexical_cast.hpp> 
 
#include <iostream> 
#include <algorithm> 
 
using namespace std; 
using namespace wrap; 
using namespace BOOST_SPIRIT_CLASSIC_NS; 
namespace bl = boost::lambda; 
namespace fs = boost::filesystem; 
 
/* ************************************************************************* */ 
// We parse an interface file into a Module object. 
// The grammar is defined using the boost/spirit combinatorial parser. 
// For example, str_p("const") parses the string "const", and the >> 
// operator creates a sequence parser. The grammar below, composed of rules 
// and with start rule [class_p], doubles as the specs for our interface files. 
/* ************************************************************************* */ 
 
/* ************************************************************************* */ 
// If a number of template arguments were given, generate a number of expanded
// class names, e.g., PriorFactor -> PriorFactorPose2, and add those classes
static void handle_possible_template(vector<Class>& classes, const Class& cls,
    const Template& t) {
  if (cls.templateArgs.empty() || t.empty()) {
    classes.push_back(cls);
  } else {
    if (cls.templateArgs.size() != 1)
      throw std::runtime_error(
          "In-line template instantiations only handle a single template argument");
    string arg = cls.templateArgs.front();
    vector<Class> classInstantiations =
        (t.nrValues() > 0) ? cls.expandTemplate(arg, t.argValues()) :
            cls.expandTemplate(arg, t.intList());
    for(const Class& c: classInstantiations)
      classes.push_back(c);
  }
}

/* ************************************************************************* */
Module::Module(const std::string& moduleName, bool enable_verbose)
: name(moduleName), verbose(enable_verbose)
{
}

/* ************************************************************************* */ 
Module::Module(const string& interfacePath, 
         const string& moduleName, bool enable_verbose)
: name(moduleName), verbose(enable_verbose)
{ 
  // read interface file
  string interfaceFile = interfacePath + "/" + moduleName + ".h";
  string contents = file_contents(interfaceFile);

  // execute parsing
  parseMarkup(contents);
}

/* ************************************************************************* */
void Module::parseMarkup(const std::string& data) {
  // The parse imperatively :-( updates variables gradually during parse
  // The one with postfix 0 are used to reset the variables after parse. 
 
  //---------------------------------------------------------------------------- 
  // Grammar with actions that build the Class object. Actions are 
  // defined within the square brackets [] and are executed whenever a 
  // rule is successfully parsed. Define BOOST_SPIRIT_DEBUG to debug. 
  // The grammar is allows a very restricted C++ header 
  // lexeme_d turns off white space skipping 
  // http://www.boost.org/doc/libs/1_37_0/libs/spirit/classic/doc/directives.html 
  // ---------------------------------------------------------------------------- 
 
  // Define Rule and instantiate basic rules
  typedef rule<phrase_scanner_t> Rule;
  BasicRules<phrase_scanner_t> basic;

  vector<string> namespaces; // current namespace tag

  // parse a full class
  Class cls0(verbose),cls(verbose);
  Template classTemplate;
  ClassGrammar class_g(cls,classTemplate);
  Rule class_p = class_g //
      [assign_a(cls.namespaces_, namespaces)]
      [bl::bind(&handle_possible_template, bl::var(classes), bl::var(cls),
          bl::var(classTemplate))]
          [clear_a(classTemplate)] //
      [assign_a(cls,cls0)];

  // parse "gtsam::Pose2" and add to singleInstantiation.typeList
  TemplateInstantiationTypedef singleInstantiation, singleInstantiation0;
  TypeListGrammar<'<','>'> typelist_g(singleInstantiation.typeList);
 
  // typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> RangeFactorPosePoint2;
  TypeGrammar instantiationClass_g(singleInstantiation.class_);
  Rule templateSingleInstantiation_p = 
    (str_p("typedef") >> instantiationClass_g >>
    typelist_g >>
    basic.className_p[assign_a(singleInstantiation.name_)] >>
    ';') 
    [assign_a(singleInstantiation.namespaces_, namespaces)]
    [push_back_a(templateInstantiationTypedefs, singleInstantiation)] 
    [assign_a(singleInstantiation, singleInstantiation0)]; 
 
  // Create grammar for global functions
  GlobalFunctionGrammar global_function_g(global_functions,namespaces);
 
  Rule include_p = str_p("#include") >> ch_p('<') >> (*(anychar_p - '>'))[push_back_a(includes)] >> ch_p('>');

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wuninitialized"
#endif

  Rule namespace_def_p =
      (str_p("namespace")
      >> basic.namespace_p[push_back_a(namespaces)]
      >> ch_p('{')
      >> *(include_p | class_p | templateSingleInstantiation_p | global_function_g | namespace_def_p | basic.comments_p)
      >> ch_p('}'))
      [pop_a(namespaces)];

#ifdef __clang__
#pragma clang diagnostic pop
#endif

  // parse forward declaration
  ForwardDeclaration fwDec0, fwDec;
  Rule forward_declaration_p =
      !(str_p("virtual")[assign_a(fwDec.isVirtual, true)]) 
      >> str_p("class") 
      >> (*(basic.namespace_p >> str_p("::")) >> basic.className_p)[assign_a(fwDec.name)]
      >> ch_p(';') 
      [push_back_a(forward_declarations, fwDec)] 
      [assign_a(cls,cls0)] // also clear class to avoid partial parse
      [assign_a(fwDec, fwDec0)]; 
 
  Rule module_content_p = basic.comments_p | include_p | class_p
      | templateSingleInstantiation_p | forward_declaration_p
      | global_function_g | namespace_def_p;
 
  Rule module_p = *module_content_p >> !end_p; 
 
  // and parse contents
  parse_info<const char*> info = parse(data.c_str(), module_p, space_p);
  if(!info.full) {
    printf("parsing stopped at \n%.20s\n",info.stop);
    cout << "Stopped in:\n"
      "class '" << cls.name_ << "'" << endl;
    throw ParseFailed((int)info.length);
  }

  // Post-process classes for serialization markers
  for(Class& cls: classes)
    cls.erase_serialization();

  // Explicitly add methods to the classes from parents so it shows in documentation
  for(Class& cls: classes)
    cls.appendInheritedMethods(cls, classes);

  // Expand templates - This is done first so that template instantiations are
  // counted in the list of valid types, have their attributes and dependencies
  // checked, etc.
  expandedClasses = ExpandTypedefInstantiations(classes,
      templateInstantiationTypedefs);

  // Dependency check list
  vector<string> validTypes = GenerateValidTypes(expandedClasses,
      forward_declarations);

  // Check that all classes have been defined somewhere
  verifyArguments<GlobalFunction>(validTypes, global_functions);
  verifyReturnTypes<GlobalFunction>(validTypes, global_functions);

  hasSerialiable = false;
  for(const Class& cls: expandedClasses)
    cls.verifyAll(validTypes,hasSerialiable);

  // Create type attributes table and check validity
  typeAttributes.addClasses(expandedClasses);
  typeAttributes.addForwardDeclarations(forward_declarations);
  // add Eigen types as template arguments are also checked ?
  vector<ForwardDeclaration> eigen;
  eigen.push_back(ForwardDeclaration("Vector"));
  eigen.push_back(ForwardDeclaration("Matrix"));
  typeAttributes.addForwardDeclarations(eigen);
  typeAttributes.checkValidity(expandedClasses);
} 
 
/* ************************************************************************* */ 
void Module::matlab_code(const string& toolboxPath) const {

  fs::create_directories(toolboxPath);

  // create the unified .cpp switch file
  const string wrapperName = name + "_wrapper";
  string wrapperFileName = toolboxPath + "/" + wrapperName + ".cpp";
  FileWriter wrapperFile(wrapperFileName, verbose, "//");
  wrapperFile.oss << "#include <wrap/matlab.h>\n";
  wrapperFile.oss << "#include <map>\n";
  wrapperFile.oss << "\n";

  // Include boost.serialization archive headers before other class headers
  if (hasSerialiable) {
    wrapperFile.oss << "#include <boost/serialization/export.hpp>\n";
    wrapperFile.oss << "#include <boost/archive/text_iarchive.hpp>\n";
    wrapperFile.oss << "#include <boost/archive/text_oarchive.hpp>\n\n";
  }

  // Generate includes while avoiding redundant includes
  generateIncludes(wrapperFile);

  // create typedef classes - we put this at the top of the wrap file so that
  // collectors and method arguments can use these typedefs
  for(const Class& cls: expandedClasses)
    if(!cls.typedefName.empty())
      wrapperFile.oss << cls.getTypedef() << "\n";
  wrapperFile.oss << "\n";

  // Generate boost.serialization export flags (needs typedefs from above)
  if (hasSerialiable) {
    for(const Class& cls: expandedClasses)
      if(cls.isSerializable)
        wrapperFile.oss << cls.getSerializationExport() << "\n";
    wrapperFile.oss << "\n";
  }

  // Generate collectors and cleanup function to be called from mexAtExit
  WriteCollectorsAndCleanupFcn(wrapperFile, name, expandedClasses);

  // generate RTTI registry (for returning derived-most types)
  WriteRTTIRegistry(wrapperFile, name, expandedClasses);

  vector<string> functionNames; // Function names stored by index for switch

  // create proxy class and wrapper code
  for(const Class& cls: expandedClasses)
    cls.matlab_proxy(toolboxPath, wrapperName, typeAttributes, wrapperFile, functionNames);

  // create matlab files and wrapper code for global functions
  for(const GlobalFunctions::value_type& p: global_functions)
    p.second.matlab_proxy(toolboxPath, wrapperName, typeAttributes, wrapperFile, functionNames);

  // finish wrapper file
  wrapperFile.oss << "\n";
  finish_wrapper(wrapperFile, functionNames);

  wrapperFile.emit(true);
}

/* ************************************************************************* */ 
void Module::generateIncludes(FileWriter& file) const {

  // collect includes
  vector<string> all_includes(includes);

  // sort and remove duplicates
  sort(all_includes.begin(), all_includes.end());
  vector<string>::const_iterator last_include = unique(all_includes.begin(), all_includes.end());
  vector<string>::const_iterator it = all_includes.begin();
  // add includes to file
  for (; it != last_include; ++it)
    file.oss << "#include <" << *it << ">" << endl;
  file.oss << "\n";
}


/* ************************************************************************* */
  void Module::finish_wrapper(FileWriter& file, const std::vector<std::string>& functionNames) const { 
    file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n"; 
    file.oss << "{\n"; 
    file.oss << "  mstream mout;\n"; // Send stdout to MATLAB console 
    file.oss << "  std::streambuf *outbuf = std::cout.rdbuf(&mout);\n\n"; 
    file.oss << "  _" << name << "_RTTIRegister();\n\n"; 
    file.oss << "  int id = unwrap<int>(in[0]);\n\n"; 
    file.oss << "  try {\n"; 
    file.oss << "    switch(id) {\n"; 
    for(size_t id = 0; id < functionNames.size(); ++id) { 
      file.oss << "    case " << id << ":\n"; 
      file.oss << "      " << functionNames[id] << "(nargout, out, nargin-1, in+1);\n"; 
      file.oss << "      break;\n"; 
    } 
    file.oss << "    }\n"; 
    file.oss << "  } catch(const std::exception& e) {\n"; 
    file.oss << "    mexErrMsgTxt((\"Exception from gtsam:\\n\" + std::string(e.what()) + \"\\n\").c_str());\n"; 
    file.oss << "  }\n"; 
    file.oss << "\n"; 
    file.oss << "  std::cout.rdbuf(outbuf);\n"; // Restore cout 
    file.oss << "}\n"; 
  } 
 
/* ************************************************************************* */ 
vector<Class> Module::ExpandTypedefInstantiations(const vector<Class>& classes, const vector<TemplateInstantiationTypedef> instantiations) { 
 
  vector<Class> expandedClasses = classes; 
 
  for(const TemplateInstantiationTypedef& inst: instantiations) {
    // Add the new class to the list 
    expandedClasses.push_back(inst.findAndExpand(classes)); 
  } 
 
  // Remove all template classes 
  for(size_t i = 0; i < expandedClasses.size(); ++i) 
    if(!expandedClasses[size_t(i)].templateArgs.empty()) { 
      expandedClasses.erase(expandedClasses.begin() + size_t(i)); 
      -- i; 
    } 
 
  return expandedClasses; 
} 
 
/* ************************************************************************* */ 
vector<string> Module::GenerateValidTypes(const vector<Class>& classes, const vector<ForwardDeclaration> forwardDeclarations) { 
  vector<string> validTypes; 
  for(const ForwardDeclaration& fwDec: forwardDeclarations) {
    validTypes.push_back(fwDec.name);
  } 
  validTypes.push_back("void"); 
  validTypes.push_back("string"); 
  validTypes.push_back("int"); 
  validTypes.push_back("bool"); 
  validTypes.push_back("char"); 
  validTypes.push_back("unsigned char"); 
  validTypes.push_back("size_t"); 
  validTypes.push_back("double"); 
  validTypes.push_back("Vector"); 
  validTypes.push_back("Matrix"); 
  //Create a list of parsed classes for dependency checking 
  for(const Class& cls: classes) {
    validTypes.push_back(cls.qualifiedName("::")); 
  } 
 
  return validTypes; 
} 
 
/* ************************************************************************* */ 
void Module::WriteCollectorsAndCleanupFcn(FileWriter& wrapperFile, const std::string& moduleName, const std::vector<Class>& classes) { 
  // Generate all collectors 
  for(const Class& cls: classes) {
    const string matlabUniqueName = cls.qualifiedName(), 
      cppName = cls.qualifiedName("::"); 
    wrapperFile.oss << "typedef std::set<boost::shared_ptr<" << cppName << ">*> " 
      << "Collector_" << matlabUniqueName << ";\n"; 
    wrapperFile.oss << "static Collector_" << matlabUniqueName << 
      " collector_" << matlabUniqueName << ";\n"; 
  } 
 
  // generate mexAtExit cleanup function 
  wrapperFile.oss << 
    "\nvoid _deleteAllObjects()\n" 
    "{\n" 
    "  mstream mout;\n" // Send stdout to MATLAB console 
    "  std::streambuf *outbuf = std::cout.rdbuf(&mout);\n\n" 
    "  bool anyDeleted = false;\n"; 
  for(const Class& cls: classes) {
    const string matlabUniqueName = cls.qualifiedName(); 
    const string cppName = cls.qualifiedName("::"); 
    const string collectorType = "Collector_" + matlabUniqueName; 
    const string collectorName = "collector_" + matlabUniqueName; 
    // The extra curly-braces around the for loops work around a limitation in MSVC (existing 
    // since 2005!) preventing more than 248 blocks. 
    wrapperFile.oss << 
      "  { for(" << collectorType << "::iterator iter = " << collectorName << ".begin();\n" 
      "      iter != " << collectorName << ".end(); ) {\n" 
      "    delete *iter;\n" 
      "    " << collectorName << ".erase(iter++);\n" 
      "    anyDeleted = true;\n" 
      "  } }\n"; 
  } 
  wrapperFile.oss << 
    "  if(anyDeleted)\n" 
    "    cout <<\n" 
    "      \"WARNING:  Wrap modules with variables in the workspace have been reloaded due to\\n\"\n" 
    "      \"calling destructors, call 'clear all' again if you plan to now recompile a wrap\\n\"\n" 
    "      \"module, so that your recompiled module is used instead of the old one.\" << endl;\n" 
    "  std::cout.rdbuf(outbuf);\n" // Restore cout 
    "}\n\n"; 
} 
 
/* ************************************************************************* */ 
void Module::WriteRTTIRegistry(FileWriter& wrapperFile, const std::string& moduleName, const std::vector<Class>& classes) { 
  wrapperFile.oss << 
    "void _" << moduleName << "_RTTIRegister() {\n" 
    "  const mxArray *alreadyCreated = mexGetVariablePtr(\"global\", \"gtsam_" + moduleName + "_rttiRegistry_created\");\n" 
    "  if(!alreadyCreated) {\n" 
    "    std::map<std::string, std::string> types;\n"; 
  for(const Class& cls: classes) {
    if(cls.isVirtual) 
      wrapperFile.oss << 
      "    types.insert(std::make_pair(typeid(" << cls.qualifiedName("::") << ").name(), \"" << cls.qualifiedName(".") << "\"));\n"; 
  } 
  wrapperFile.oss << "\n"; 
 
  wrapperFile.oss << 
    "    mxArray *registry = mexGetVariable(\"global\", \"gtsamwrap_rttiRegistry\");\n" 
    "    if(!registry)\n" 
    "      registry = mxCreateStructMatrix(1, 1, 0, NULL);\n" 
    "    typedef std::pair<std::string, std::string> StringPair;\n" 
    "    for(const StringPair& rtti_matlab: types) {\n"
    "      int fieldId = mxAddField(registry, rtti_matlab.first.c_str());\n" 
    "      if(fieldId < 0)\n" 
    "        mexErrMsgTxt(\"gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly\");\n" 
    "      mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());\n" 
    "      mxSetFieldByNumber(registry, 0, fieldId, matlabName);\n" 
    "    }\n" 
    "    if(mexPutVariable(\"global\", \"gtsamwrap_rttiRegistry\", registry) != 0)\n" 
    "      mexErrMsgTxt(\"gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly\");\n" 
    "    mxDestroyArray(registry);\n" 
    "    \n" 
    "    mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);\n" 
    "    if(mexPutVariable(\"global\", \"gtsam_" + moduleName + "_rttiRegistry_created\", newAlreadyCreated) != 0)\n" 
    "      mexErrMsgTxt(\"gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly\");\n" 
    "    mxDestroyArray(newAlreadyCreated);\n" 
    "  }\n" 
    "}\n" 
    "\n"; 
} 
 
/* ************************************************************************* */ 
void Module::python_wrapper(const string& toolboxPath) const {

  fs::create_directories(toolboxPath);

  // create the unified .cpp switch file
  const string wrapperName = name + "_python";
  string wrapperFileName = toolboxPath + "/" + wrapperName + ".cpp";
  FileWriter wrapperFile(wrapperFileName, verbose, "//");
  wrapperFile.oss << "#include <boost/python.hpp>\n\n";
  wrapperFile.oss << "using namespace boost::python;\n";
  wrapperFile.oss << "BOOST_PYTHON_MODULE(" + name + ")\n";
  wrapperFile.oss << "{\n";

  // write out classes
  for(const Class& cls: expandedClasses)
    cls.python_wrapper(wrapperFile);

  // write out global functions
  for(const GlobalFunctions::value_type& p: global_functions)
    p.second.python_wrapper(wrapperFile);

  // finish wrapper file
  wrapperFile.oss << "}\n";

  wrapperFile.emit(true);
}

/* ************************************************************************* */
