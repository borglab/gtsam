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
 **/

#include "Module.h"
#include "FileWriter.h"
#include "utilities.h"
#include "spirit_actors.h"

//#define BOOST_SPIRIT_DEBUG
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/spirit/include/classic_clear_actor.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <algorithm>

using namespace std;
using namespace wrap;
using namespace BOOST_SPIRIT_CLASSIC_NS;
namespace bl = boost::lambda;
namespace fs = boost::filesystem;

typedef rule<BOOST_SPIRIT_CLASSIC_NS::phrase_scanner_t> Rule;

/* ************************************************************************* */
// We parse an interface file into a Module object.
// The grammar is defined using the boost/spirit combinatorial parser.
// For example, str_p("const") parses the string "const", and the >>
// operator creates a sequence parser. The grammar below, composed of rules
// and with start rule [class_p], doubles as the specs for our interface files.
/* ************************************************************************* */

Module::Module(const string& interfacePath,
	       const string& moduleName, bool enable_verbose) : name(moduleName), verbose(enable_verbose)
{
  // these variables will be imperatively updated to gradually build [cls]
  // The one with postfix 0 are used to reset the variables after parse.
	string methodName, methodName0;
	bool isConst, isConst0 = false;
	ReturnValue retVal0, retVal;
  Argument arg0, arg;
  ArgumentList args0, args;
  vector<string> arg_dup; ///keep track of duplicates
  Constructor constructor0(enable_verbose), constructor(enable_verbose);
  Deconstructor deconstructor0(enable_verbose), deconstructor(enable_verbose);
  //Method method0(enable_verbose), method(enable_verbose);
  StaticMethod static_method0(enable_verbose), static_method(enable_verbose);
  Class cls0(enable_verbose),cls(enable_verbose);
  vector<string> namespaces, /// current namespace tag
  							 namespace_includes, /// current set of includes
  							 namespaces_return, /// namespace for current return type
  							 using_namespace_current;  /// All namespaces from "using" declarations
  string include_path = "";
  string class_name = "";
  const string null_str = "";

  //----------------------------------------------------------------------------
  // Grammar with actions that build the Class object. Actions are
  // defined within the square brackets [] and are executed whenever a
  // rule is successfully parsed. Define BOOST_SPIRIT_DEBUG to debug.
  // The grammar is allows a very restricted C++ header
  // lexeme_d turns off white space skipping
  // http://www.boost.org/doc/libs/1_37_0/libs/spirit/classic/doc/directives.html
  // ----------------------------------------------------------------------------

  Rule comments_p =  comment_p("/*", "*/") |	comment_p("//", eol_p);

  Rule basisType_p =
    (str_p("string") | "bool" | "size_t" | "int" | "double" | "char" | "unsigned char");

  Rule keywords_p =
  	(str_p("const") | "static" | "namespace" | basisType_p);

  Rule eigenType_p =
    (str_p("Vector") | "Matrix");

  Rule className_p  = (lexeme_d[upper_p >> *(alnum_p | '_')] - eigenType_p - keywords_p);

  Rule namespace_name_p = lexeme_d[lower_p >> *(alnum_p | '_')] - keywords_p;

  Rule namespace_arg_p = namespace_name_p[push_back_a(arg.namespaces)] >> str_p("::");

  Rule argEigenType_p =
  	eigenType_p[assign_a(arg.type)] >>
  	!ch_p('*')[assign_a(arg.is_ptr,true)];

  Rule eigenRef_p =
      !str_p("const") [assign_a(arg.is_const,true)] >>
      eigenType_p     [assign_a(arg.type)] >>
      ch_p('&')       [assign_a(arg.is_ref,true)];

  Rule classArg_p =
    !str_p("const") [assign_a(arg.is_const,true)] >>
  	*namespace_arg_p >>
    className_p[assign_a(arg.type)] >>
    (ch_p('*')[assign_a(arg.is_ptr,true)] | ch_p('&')[assign_a(arg.is_ref,true)]);

	Rule classParent_p =
		*(namespace_name_p[push_back_a(cls.qualifiedParent)] >> str_p("::")) >>
		className_p[push_back_a(cls.qualifiedParent)];

  Rule name_p = lexeme_d[alpha_p >> *(alnum_p | '_')];

  Rule argument_p = 
    ((basisType_p[assign_a(arg.type)] | argEigenType_p | eigenRef_p | classArg_p)
    		>> name_p[assign_a(arg.name)])
    [push_back_a(args, arg)]
    [assign_a(arg,arg0)];

  Rule argumentList_p = !argument_p >> * (',' >> argument_p);

  Rule constructor_p = 
    (className_p >> '(' >> argumentList_p >> ')' >> ';' >> !comments_p)
    [push_back_a(constructor.args_list, args)]
    [assign_a(args,args0)];
    //[assign_a(constructor.args,args)]
    //[assign_a(constructor.name,cls.name)]
    //[push_back_a(cls.constructors, constructor)]
    //[assign_a(constructor,constructor0)];

  Rule namespace_ret_p = namespace_name_p[push_back_a(namespaces_return)] >> str_p("::");

  Rule returnType1_p =
		(basisType_p[assign_a(retVal.type1)][assign_a(retVal.category1, ReturnValue::BASIS)]) |
		((*namespace_ret_p)[assign_a(retVal.namespaces1, namespaces_return)][clear_a(namespaces_return)]
				>> (className_p[assign_a(retVal.type1)][assign_a(retVal.category1, ReturnValue::CLASS)]) >>
				!ch_p('*')[assign_a(retVal.isPtr1,true)]) |
		(eigenType_p[assign_a(retVal.type1)][assign_a(retVal.category1, ReturnValue::EIGEN)]);

  Rule returnType2_p =
		(basisType_p[assign_a(retVal.type2)][assign_a(retVal.category2, ReturnValue::BASIS)]) |
		((*namespace_ret_p)[assign_a(retVal.namespaces2, namespaces_return)][clear_a(namespaces_return)]
				>> (className_p[assign_a(retVal.type2)][assign_a(retVal.category2, ReturnValue::CLASS)]) >>
				!ch_p('*')  [assign_a(retVal.isPtr2,true)]) |
		(eigenType_p[assign_a(retVal.type2)][assign_a(retVal.category2, ReturnValue::EIGEN)]);

  Rule pair_p = 
    (str_p("pair") >> '<' >> returnType1_p >> ',' >> returnType2_p >> '>')
    [assign_a(retVal.isPair,true)];

  Rule void_p = str_p("void")[assign_a(retVal.type1)];

  Rule returnType_p = void_p | returnType1_p | pair_p;

  Rule methodName_p = lexeme_d[lower_p >> *(alnum_p | '_')];

  Rule method_p = 
    (returnType_p >> methodName_p[assign_a(methodName)] >>
     '(' >> argumentList_p >> ')' >> 
     !str_p("const")[assign_a(isConst,true)] >> ';' >> *comments_p)
		[bl::bind(&Method::addOverload,
			bl::var(cls.methods)[bl::var(methodName)],
			verbose,
			bl::var(isConst),
			bl::var(methodName),
			bl::var(args),
		  bl::var(retVal))]
	  [assign_a(isConst,isConst0)]
		[assign_a(methodName,methodName0)]
    [assign_a(args,args0)]
    [assign_a(retVal,retVal0)];

  Rule staticMethodName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

  Rule static_method_p =
    (str_p("static") >> returnType_p >> staticMethodName_p[assign_a(methodName)] >>
     '(' >> argumentList_p >> ')' >> ';' >> *comments_p)
		[bl::bind(&StaticMethod::addOverload,
			bl::var(cls.static_methods)[bl::var(methodName)],
			verbose,
			bl::var(methodName),
			bl::var(args),
		  bl::var(retVal))]
		[assign_a(methodName,methodName0)]
    [assign_a(args,args0)]
    [assign_a(retVal,retVal0)];

  Rule functions_p = constructor_p | method_p | static_method_p;

  Rule include_p = str_p("#include") >> ch_p('<') >> (*(anychar_p - '>'))[assign_a(include_path)] >> ch_p('>');

  Rule class_p =
  		(!*include_p
  		>> str_p("class")[push_back_a(cls.includes, include_path)][assign_a(include_path, null_str)]
  		>> className_p[assign_a(cls.name)]
			>> ((':' >> classParent_p >> '{') | '{') // By having (parent >> '{' | '{') here instead of (!parent >> '{'), we trigger a parse error on a badly-formed parent spec
  		>> *(functions_p | comments_p)
  		>> str_p("};"))
        [assign_a(constructor.name, cls.name)]
        [assign_a(cls.constructor, constructor)]
  		[assign_a(cls.namespaces, namespaces)]
  		 [assign_a(cls.using_namespaces, using_namespace_current)]
  		[append_a(cls.includes, namespace_includes)]
        [assign_a(deconstructor.name,cls.name)]
        [assign_a(cls.deconstructor, deconstructor)]
  		[push_back_a(classes,cls)]
        [assign_a(deconstructor,deconstructor0)]
        [assign_a(constructor, constructor0)]
  		[assign_a(cls,cls0)];

	Rule namespace_def_p =
			(!*include_p
			>> str_p("namespace")[push_back_a(namespace_includes, include_path)][assign_a(include_path, null_str)]
			>> namespace_name_p[push_back_a(namespaces)]
			>> ch_p('{')
			>> *(class_p | namespace_def_p | comments_p)
			>> str_p("}///\\namespace") // end namespace, avoid confusion with classes
			>> !namespace_name_p)
			[pop_a(namespaces)]
			[pop_a(namespace_includes)];

	Rule using_namespace_p =
			str_p("using") >> str_p("namespace")
			>> namespace_name_p[push_back_a(using_namespace_current)] >> ch_p(';');

	Rule forward_declaration_p =
			str_p("class") >>
					(*(namespace_name_p >> str_p("::")) >> className_p)[push_back_a(forward_declarations)]
					>> ch_p(';');

  Rule module_content_p =	comments_p | using_namespace_p | class_p | forward_declaration_p | namespace_def_p ;

  Rule module_p = *module_content_p >> !end_p;

  //----------------------------------------------------------------------------
  // for debugging, define BOOST_SPIRIT_DEBUG
# ifdef BOOST_SPIRIT_DEBUG
  BOOST_SPIRIT_DEBUG_NODE(className_p);
  BOOST_SPIRIT_DEBUG_NODE(classPtr_p);
  BOOST_SPIRIT_DEBUG_NODE(classRef_p);
  BOOST_SPIRIT_DEBUG_NODE(basisType_p);
  BOOST_SPIRIT_DEBUG_NODE(name_p);
  BOOST_SPIRIT_DEBUG_NODE(argument_p);
  BOOST_SPIRIT_DEBUG_NODE(argumentList_p);
  BOOST_SPIRIT_DEBUG_NODE(constructor_p);
  BOOST_SPIRIT_DEBUG_NODE(returnType1_p);
  BOOST_SPIRIT_DEBUG_NODE(returnType2_p);
  BOOST_SPIRIT_DEBUG_NODE(pair_p);
  BOOST_SPIRIT_DEBUG_NODE(void_p);
  BOOST_SPIRIT_DEBUG_NODE(returnType_p);
  BOOST_SPIRIT_DEBUG_NODE(methodName_p);
  BOOST_SPIRIT_DEBUG_NODE(method_p);
  BOOST_SPIRIT_DEBUG_NODE(class_p);
  BOOST_SPIRIT_DEBUG_NODE(namespace_def_p);
  BOOST_SPIRIT_DEBUG_NODE(module_p);
# endif
  //----------------------------------------------------------------------------

  // read interface file
  string interfaceFile = interfacePath + "/" + moduleName + ".h";
  string contents = file_contents(interfaceFile);

  // and parse contents
  parse_info<const char*> info = parse(contents.c_str(), module_p, space_p);
  if(!info.full) {
    printf("parsing stopped at \n%.20s\n",info.stop);
    throw ParseFailed(info.length);
  }
}

/* ************************************************************************* */
template<class T>
void verifyArguments(const vector<string>& validArgs, const map<string,T>& vt) {
	typedef typename map<string,T>::value_type Name_Method;
	BOOST_FOREACH(const Name_Method& name_method, vt) {
		const T& t = name_method.second;
		BOOST_FOREACH(const ArgumentList& argList, t.argLists) {
			BOOST_FOREACH(Argument arg, argList) {
				string fullType = arg.qualifiedType("::");
				if(find(validArgs.begin(), validArgs.end(), fullType)
					== validArgs.end())
					throw DependencyMissing(fullType, t.name);
			}
		}
	}
}

/* ************************************************************************* */
template<class T>
void verifyReturnTypes(const vector<string>& validtypes, const map<string,T>& vt) {
	typedef typename map<string,T>::value_type Name_Method;
	BOOST_FOREACH(const Name_Method& name_method, vt) {
		const T& t = name_method.second;
		BOOST_FOREACH(const ReturnValue& retval, t.returnVals) {
			if (find(validtypes.begin(), validtypes.end(), retval.qualifiedType1("::"))	== validtypes.end())
				throw DependencyMissing(retval.qualifiedType1("::"), t.name);
			if (retval.isPair && find(validtypes.begin(), validtypes.end(), retval.qualifiedType2("::"))	== validtypes.end())
				throw DependencyMissing(retval.qualifiedType2("::"), t.name);
		}
	}
}

/* ************************************************************************* */
void Module::matlab_code(const string& toolboxPath, const string& headerPath) const {

    fs::create_directories(toolboxPath);

		// create the unified .cpp switch file
		const string wrapperName = name + "_wrapper";
		string wrapperFileName = toolboxPath + "/" + wrapperName + ".cpp";
		FileWriter wrapperFile(wrapperFileName, verbose, "//");
		vector<string> functionNames; // Function names stored by index for switch
		wrapperFile.oss << "#include <wrap/matlab.h>\n";
		wrapperFile.oss << "#include <map>\n";
		wrapperFile.oss << "#include <boost/foreach.hpp>\n";
		wrapperFile.oss << "\n";

    // Dependency check list
    vector<string> validTypes = forward_declarations;
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
    BOOST_FOREACH(Class cls, classes) {
			validTypes.push_back(cls.qualifiedName("::"));
    }

		// Generate all includes
		BOOST_FOREACH(Class cls, classes) {
			generateIncludes(wrapperFile, cls.name, cls.includes);
		}
		wrapperFile.oss << "\n";

		// Generate all collectors
		BOOST_FOREACH(Class cls, classes) {
			const string matlabName = cls.qualifiedName(), cppName = cls.qualifiedName("::");
			wrapperFile.oss << "typedef std::set<boost::shared_ptr<" << cppName << ">*> "
				<< "Collector_" << matlabName << ";\n";
			wrapperFile.oss << "static Collector_" << matlabName <<
				" collector_" << matlabName << ";\n";
		}

		// generate mexAtExit cleanup function
		wrapperFile.oss << "void _deleteAllObjects()\n";
		wrapperFile.oss << "{\n";
		BOOST_FOREACH(Class cls, classes) {
			const string matlabName = cls.qualifiedName();
			const string cppName = cls.qualifiedName("::");
			const string collectorType = "Collector_" + matlabName;
			const string collectorName = "collector_" + matlabName;
			wrapperFile.oss << "  for(" << collectorType << "::iterator iter = " << collectorName << ".begin();\n";
			wrapperFile.oss << "      iter != " << collectorName << ".end(); ) {\n";
			wrapperFile.oss << "    delete *iter;\n";
			wrapperFile.oss << "    " << collectorName << ".erase(iter++);\n";
			wrapperFile.oss << "  }\n";
		}
		wrapperFile.oss << "}\n";

    // generate proxy classes and wrappers
    BOOST_FOREACH(Class cls, classes) {
      // create proxy class and wrapper code
      string classFile = toolboxPath + "/" + cls.qualifiedName() + ".m";
      cls.matlab_proxy(classFile, wrapperName, wrapperFile, functionNames);

      // verify all of the function arguments
      //TODO:verifyArguments<ArgumentList>(validTypes, cls.constructor.args_list);
      verifyArguments<StaticMethod>(validTypes, cls.static_methods);
      verifyArguments<Method>(validTypes, cls.methods);

      // verify function return types
      verifyReturnTypes<StaticMethod>(validTypes, cls.static_methods);
      verifyReturnTypes<Method>(validTypes, cls.methods);
    }  

		// finish wrapper file
		finish_wrapper(wrapperFile, functionNames);

		wrapperFile.emit(true);
  }

/* ************************************************************************* */
	void Module::finish_wrapper(FileWriter& file, const std::vector<std::string>& functionNames) const {
		file.oss << "void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n";
		file.oss << "{\n";
		file.oss << "  mstream mout;\n"; // Send stdout to MATLAB console, see matlab.h
		file.oss << "  std::streambuf *outbuf = std::cout.rdbuf(&mout);\n\n";
		file.oss << "  int id = unwrap<int>(in[0]);\n\n";
		file.oss << "  switch(id) {\n";
		for(size_t id = 0; id < functionNames.size(); ++id) {
			file.oss << "  case " << id << ":\n";
			file.oss << "    " << functionNames[id] << "(nargout, out, nargin-1, in+1);\n";
			file.oss << "    break;\n";
		}
		file.oss << "  }\n";
		file.oss << "\n";
		file.oss << "  std::cout.rdbuf(outbuf);\n"; // Restore cout, see matlab.h
		file.oss << "}\n";
	}

/* ************************************************************************* */
