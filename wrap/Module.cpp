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
#include "spirit_actors.h" 
 
//#define BOOST_SPIRIT_DEBUG 
#include <boost/spirit/include/classic_confix.hpp> 
#include <boost/spirit/include/classic_clear_actor.hpp> 
#include <boost/spirit/include/classic_insert_at_actor.hpp> 
#include <boost/lambda/bind.hpp> 
#include <boost/lambda/lambda.hpp> 
#include <boost/lambda/construct.hpp> 
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
 
/* ************************************************************************* */ 
void handle_possible_template(vector<Class>& classes, const Class& cls, const string& templateArgument, const vector<vector<string> >& instantiations) { 
	if(instantiations.empty()) { 
		classes.push_back(cls); 
	} else { 
		vector<Class> classInstantiations = cls.expandTemplate(templateArgument, instantiations); 
		BOOST_FOREACH(const Class& c, classInstantiations) { 
			classes.push_back(c); 
		} 
	} 
} 
 
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
  StaticMethod static_method0(enable_verbose), static_method(enable_verbose); 
  Class cls0(enable_verbose),cls(enable_verbose); 
  GlobalFunction globalFunc0(enable_verbose), globalFunc(enable_verbose); 
  ForwardDeclaration fwDec0, fwDec; 
  vector<string> namespaces, /// current namespace tag 
  							 namespaces_return; /// namespace for current return type 
	string templateArgument; 
	vector<string> templateInstantiationNamespace; 
	vector<vector<string> > templateInstantiations; 
	TemplateInstantiationTypedef singleInstantiation, singleInstantiation0; 
  string include_path = ""; 
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
 
	Rule name_p = lexeme_d[alpha_p >> *(alnum_p | '_')]; 
 
	Rule classParent_p = 
		*(namespace_name_p[push_back_a(cls.qualifiedParent)] >> str_p("::")) >> 
		className_p[push_back_a(cls.qualifiedParent)]; 
 
	Rule templateInstantiation_p = 
		(*(namespace_name_p[push_back_a(templateInstantiationNamespace)] >> str_p("::")) >> 
		className_p[push_back_a(templateInstantiationNamespace)]) 
		[push_back_a(templateInstantiations, templateInstantiationNamespace)] 
	  [clear_a(templateInstantiationNamespace)]; 
 
	Rule templateInstantiations_p = 
		(str_p("template") >> 
		'<' >> name_p[assign_a(templateArgument)] >> '=' >> '{' >> 
		!(templateInstantiation_p >> *(',' >> templateInstantiation_p)) >> 
		'}' >> '>') 
		[push_back_a(cls.templateArgs, templateArgument)]; 
 
	Rule templateSingleInstantiationArg_p = 
		(*(namespace_name_p[push_back_a(templateInstantiationNamespace)] >> str_p("::")) >> 
		className_p[push_back_a(templateInstantiationNamespace)]) 
		[push_back_a(singleInstantiation.typeList, templateInstantiationNamespace)] 
	  [clear_a(templateInstantiationNamespace)]; 
 
	Rule templateSingleInstantiation_p = 
		(str_p("typedef") >> 
		*(namespace_name_p[push_back_a(singleInstantiation.classNamespaces)] >> str_p("::")) >> 
		className_p[assign_a(singleInstantiation.className)] >> 
		'<' >> templateSingleInstantiationArg_p >> *(',' >> templateSingleInstantiationArg_p) >> 
		'>' >> 
		className_p[assign_a(singleInstantiation.name)] >> 
		';') 
		[assign_a(singleInstantiation.namespaces, namespaces)] 
	  [push_back_a(templateInstantiationTypedefs, singleInstantiation)] 
		[assign_a(singleInstantiation, singleInstantiation0)]; 
 
	Rule templateList_p = 
		(str_p("template") >> 
		'<' >> name_p[push_back_a(cls.templateArgs)] >> *(',' >> name_p[push_back_a(cls.templateArgs)]) >> 
		'>'); 
 
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
 
  Rule class_p = 
		  (str_p("")[assign_a(cls,cls0)]) 
  		>> (!(templateInstantiations_p | templateList_p) 
			>> !(str_p("virtual")[assign_a(cls.isVirtual, true)]) 
  		>> str_p("class") 
  		>> className_p[assign_a(cls.name)] 
			>> ((':' >> classParent_p >> '{') | '{') 
  		>> *(functions_p | comments_p) 
  		>> str_p("};")) 
      [assign_a(constructor.name, cls.name)] 
      [assign_a(cls.constructor, constructor)] 
  		[assign_a(cls.namespaces, namespaces)] 
      [assign_a(deconstructor.name,cls.name)] 
      [assign_a(cls.deconstructor, deconstructor)] 
			[bl::bind(&handle_possible_template, bl::var(classes), bl::var(cls), bl::var(templateArgument), bl::var(templateInstantiations))] 
      [assign_a(deconstructor,deconstructor0)] 
      [assign_a(constructor, constructor0)] 
  		[assign_a(cls,cls0)] 
			[clear_a(templateArgument)] 
			[clear_a(templateInstantiations)]; 
 
  Rule global_function_p = 
      (returnType_p >> staticMethodName_p[assign_a(methodName)] >> 
       '(' >> argumentList_p >> ')' >> ';' >> *comments_p) 
  		[bl::bind(&GlobalFunction::addOverload, 
  			bl::var(global_functions)[bl::var(methodName)], 
  			verbose, 
  			bl::var(methodName), 
  			bl::var(args), 
  		  bl::var(retVal), 
  		  bl::var(namespaces))] 
  		[assign_a(methodName,methodName0)] 
      [assign_a(args,args0)] 
      [assign_a(retVal,retVal0)]; 
 
  Rule include_p = str_p("#include") >> ch_p('<') >> (*(anychar_p - '>'))[push_back_a(includes)] >> ch_p('>'); 
 
	Rule namespace_def_p = 
			(str_p("namespace") 
			>> namespace_name_p[push_back_a(namespaces)] 
			>> ch_p('{') 
			>> *(include_p | class_p | templateSingleInstantiation_p | global_function_p | namespace_def_p | comments_p) 
			>> ch_p('}')) 
			[pop_a(namespaces)]; 
 
	Rule forward_declaration_p = 
			!(str_p("virtual")[assign_a(fwDec.isVirtual, true)]) 
			>> str_p("class") 
			>> (*(namespace_name_p >> str_p("::")) >> className_p)[assign_a(fwDec.name)] 
			>> ch_p(';') 
			[push_back_a(forward_declarations, fwDec)] 
			[assign_a(fwDec, fwDec0)]; 
 
  Rule module_content_p =	comments_p | include_p | class_p | templateSingleInstantiation_p | forward_declaration_p | global_function_p | namespace_def_p; 
 
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
    throw ParseFailed((int)info.length); 
  } 

  //Explicitly add methods to the classes from parents so it shows in documentation
  BOOST_FOREACH(Class& cls, classes)
  {
    map<string, Method> inhereted = appendInheretedMethods(cls, classes);
    cls.methods.insert(inhereted.begin(), inhereted.end());
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
 
		// Expand templates - This is done first so that template instantiations are 
		// counted in the list of valid types, have their attributes and dependencies 
		// checked, etc. 
		vector<Class> expandedClasses = ExpandTypedefInstantiations(classes, templateInstantiationTypedefs); 
 
    // Dependency check list 
    vector<string> validTypes = GenerateValidTypes(expandedClasses, forward_declarations); 
 
		// Check that all classes have been defined somewhere 
		verifyArguments<GlobalFunction>(validTypes, global_functions); 
		verifyReturnTypes<GlobalFunction>(validTypes, global_functions); 
 
		BOOST_FOREACH(const Class& cls, expandedClasses) { 
			// verify all of the function arguments 
			//TODO:verifyArguments<ArgumentList>(validTypes, cls.constructor.args_list); 
			verifyArguments<StaticMethod>(validTypes, cls.static_methods); 
			verifyArguments<Method>(validTypes, cls.methods); 
 
			// verify function return types 
			verifyReturnTypes<StaticMethod>(validTypes, cls.static_methods); 
			verifyReturnTypes<Method>(validTypes, cls.methods); 
 
			// verify parents 
			if(!cls.qualifiedParent.empty() && std::find(validTypes.begin(), validTypes.end(), wrap::qualifiedName("::", cls.qualifiedParent)) == validTypes.end()) 
				throw DependencyMissing(wrap::qualifiedName("::", cls.qualifiedParent), cls.qualifiedName("::")); 

		} 
 
		// Create type attributes table and check validity 
		TypeAttributesTable typeAttributes; 
		typeAttributes.addClasses(expandedClasses); 
		typeAttributes.addForwardDeclarations(forward_declarations); 
		typeAttributes.checkValidity(expandedClasses); 
 
		// Generate includes while avoiding redundant includes 
		generateIncludes(wrapperFile); 
 
		// create typedef classes - we put this at the top of the wrap file so that collectors and method arguments can use these typedefs 
		BOOST_FOREACH(const Class& cls, expandedClasses) { 
			if(!cls.typedefName.empty()) 
				wrapperFile.oss << cls.getTypedef() << "\n"; 
		} 
		wrapperFile.oss << "\n"; 
 
		// Generate collectors and cleanup function to be called from mexAtExit 
		WriteCollectorsAndCleanupFcn(wrapperFile, name, expandedClasses); 
 
		// generate RTTI registry (for returning derived-most types) 
		WriteRTTIRegistry(wrapperFile, name, expandedClasses); 
 
		// create proxy class and wrapper code 
		BOOST_FOREACH(const Class& cls, expandedClasses) { 
      cls.matlab_proxy(toolboxPath, wrapperName, typeAttributes, wrapperFile, functionNames); 
    }   
 
		// create matlab files and wrapper code for global functions 
		BOOST_FOREACH(const GlobalFunctions::value_type& p, global_functions) { 
			p.second.matlab_proxy(toolboxPath, wrapperName, typeAttributes, wrapperFile, functionNames); 
		} 
 
		// finish wrapper file 
		wrapperFile.oss << "\n"; 
		finish_wrapper(wrapperFile, functionNames); 
 
		wrapperFile.emit(true); 
  } 
/* ************************************************************************* */ 
map<string, Method> Module::appendInheretedMethods(const Class& cls, const vector<Class>& classes)
{
    map<string, Method> methods;
    if(!cls.qualifiedParent.empty())
    {
        cout << "Class: " << cls.name << " Parent Name: " << cls.qualifiedParent.back() << endl;
        //Find Class
        BOOST_FOREACH(const Class& parent, classes)
        {
            //We found the class for our parent
            if(parent.name == cls.qualifiedParent.back())
            {
                cout << "Inner class: " << cls.qualifiedParent.back() << endl;
                Methods inhereted = appendInheretedMethods(parent, classes);
                methods.insert(inhereted.begin(), inhereted.end());
            }
        }
    }
    else
    {
        cout << "Dead end: " << cls.name << endl;
        methods.insert(cls.methods.begin(), cls.methods.end());
    }

    return methods;
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
 
	BOOST_FOREACH(const TemplateInstantiationTypedef& inst, instantiations) { 
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
	BOOST_FOREACH(const ForwardDeclaration& fwDec, forwardDeclarations) { 
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
	BOOST_FOREACH(const Class& cls, classes) { 
		validTypes.push_back(cls.qualifiedName("::")); 
	} 
 
	return validTypes; 
} 
 
/* ************************************************************************* */ 
void Module::WriteCollectorsAndCleanupFcn(FileWriter& wrapperFile, const std::string& moduleName, const std::vector<Class>& classes) { 
	// Generate all collectors 
	BOOST_FOREACH(const Class& cls, classes) { 
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
	BOOST_FOREACH(const Class& cls, classes) { 
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
	BOOST_FOREACH(const Class& cls, classes) { 
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
		"    BOOST_FOREACH(const StringPair& rtti_matlab, types) {\n" 
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
