/**
 * file: Module.ccp
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>

//#define BOOST_SPIRIT_DEBUG
#include <boost/spirit/include/classic_core.hpp>
#include <boost/foreach.hpp>

#include "Module.h"
#include "utilities.h"

using namespace std;
using namespace BOOST_SPIRIT_CLASSIC_NS;

typedef rule<BOOST_SPIRIT_CLASSIC_NS::phrase_scanner_t> Rule;

/* ************************************************************************* */
// We parse an interface file into a Module object.
// The grammar is defined using the boost/spirit combinatorial parser.
// For example, str_p("const") parses the string "const", and the >>
// operator creates a sequence parser. The grammar below, composed of rules
// and with start rule [class_p], doubles as the specs for our interface files.
/* ************************************************************************* */

Module::Module(const string& interfacePath,
	       const string& moduleName) : name(moduleName)
{
  // these variables will be imperatively updated to gradually build [cls]
  // The one with postfix 0 are used to reset the variables after parse.
  Argument arg0, arg;
  ArgumentList args0, args;
  Constructor constructor0, constructor;
  Method method0, method;
  Class cls0,cls;

  //----------------------------------------------------------------------------
  // Grammar with actions that build the Class object. Actions are
  // defined within the square brackets [] and are executed whenever a
  // rule is successfully parsed. Define BOOST_SPIRIT_DEBUG to debug.
  // The grammar is allows a very restricted C++ header:
  // - No comments allowed.
  //  -Only types allowed are string, bool, size_t int, double, Vector, and Matrix
  //   as well as class names that start with an uppercase letter
  // - The types unsigned int and bool should be specified as int.
  // ----------------------------------------------------------------------------

  // lexeme_d turns off white space skipping
  // http://www.boost.org/doc/libs/1_37_0/libs/spirit/classic/doc/directives.html

  Rule className_p  = lexeme_d[upper_p >> *(alnum_p | '_')];

  Rule classPtr_p =
    className_p     [assign_a(arg.type)] >> 
    ch_p('*')       [assign_a(arg.is_ptr,true)];

  Rule classRef_p =
    !str_p("const") [assign_a(arg.is_const,true)] >> 
    className_p     [assign_a(arg.type)] >> 
    ch_p('&')       [assign_a(arg.is_ref,true)];

  Rule basisType_p = 
    (str_p("string") | "bool" | "size_t" | "int" | "double");

  Rule ublasType = 
    (str_p("Vector") | "Matrix")[assign_a(arg.type)] >>
    !ch_p('*')[assign_a(arg.is_ptr,true)];

  Rule name_p = lexeme_d[alpha_p >> *(alnum_p | '_')];

  Rule argument_p = 
    ((basisType_p[assign_a(arg.type)] | ublasType | classPtr_p | classRef_p) >> name_p[assign_a(arg.name)])
    [push_back_a(args, arg)]
    [assign_a(arg,arg0)];

  Rule argumentList_p = !argument_p >> * (',' >> argument_p);

  Rule constructor_p = 
    (className_p >> '(' >> argumentList_p >> ')' >> ';')
    [assign_a(constructor.args,args)]
    [assign_a(args,args0)]
    [push_back_a(cls.constructors, constructor)]
    [assign_a(constructor,constructor0)];

  Rule returnType1_p =
    basisType_p[assign_a(method.returns)] | 
    ((className_p | "Vector" | "Matrix")[assign_a(method.returns)] >>
     !ch_p('*')  [assign_a(method.returns_ptr,true)]);

  Rule returnType2_p =
    basisType_p[assign_a(method.returns2)] | 
    ((className_p | "Vector" | "Matrix")[assign_a(method.returns2)] >>
     !ch_p('*')  [assign_a(method.returns_ptr2,true)]);

  Rule pair_p = 
    (str_p("pair") >> '<' >> returnType1_p >> ',' >> returnType2_p >> '>')
    [assign_a(method.returns_pair,true)];

  Rule void_p = str_p("void")[assign_a(method.returns)];

  Rule returnType_p = void_p | returnType1_p | pair_p;

  Rule methodName_p = lexeme_d[lower_p >> *(alnum_p | '_')];

  Rule method_p = 
    (returnType_p >> methodName_p[assign_a(method.name)] >> 
     '(' >> argumentList_p >> ')' >> 
     !str_p("const")[assign_a(method.is_const,true)] >> ';')
    [assign_a(method.args,args)]
    [assign_a(args,args0)]
    [push_back_a(cls.methods, method)]
    [assign_a(method,method0)];

  Rule class_p = str_p("class") >> className_p[assign_a(cls.name)] >> '{' >> 
    *constructor_p >> 
    *method_p >> 
    '}' >> ";";

  Rule module_p = +class_p
    [push_back_a(classes,cls)]
    [assign_a(cls,cls0)] 
    >> !end_p;

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
  BOOST_SPIRIT_DEBUG_NODE(module_p);
# endif
  //----------------------------------------------------------------------------

  // read interface file
  string interfaceFile = interfacePath + "/" + moduleName + ".h";
  string contents = file_contents(interfaceFile);

  // Comment parser : does not work for some reason
  rule<> comment_p = str_p("/*") >> +anychar_p >> "*/";
  rule<> skip_p = space_p; // | comment_p;

  // and parse contents
  parse_info<const char*> info = parse(contents.c_str(), module_p, skip_p); 
  if(!info.full) {
    printf("parsing stopped at \n%.20s\n",info.stop);
    throw ParseFailed(info.length);
  }
}

/* ************************************************************************* */
void Module::matlab_code(const string& toolboxPath, 
			 const string& nameSpace, 
			 const string& mexFlags) 
{
  try {
    string installCmd = "install -d " + toolboxPath;
    system(installCmd.c_str());

    // create make m-file
    string makeFile = toolboxPath + "/make_" + name + ".m";
    ofstream ofs(makeFile.c_str());
    if(!ofs) throw CantOpenFile(makeFile);

    cerr << "generating " << makeFile << endl;
    emit_header_comment(ofs,"%");
    ofs << "echo on" << endl << endl;
    ofs << "toolboxpath = pwd" << endl;
    ofs << "addpath(toolboxpath);" << endl << endl;

    // generate proxy classes and wrappers
    BOOST_FOREACH(Class cls, classes) {
      // create directory if needed
      string classPath = toolboxPath + "/@" + cls.name;
      string installCmd = "install -d " + classPath;
      system(installCmd.c_str());

      // create proxy class
      string classFile = classPath + "/" + cls.name + ".m";
      cls.matlab_proxy(classFile);

      // create constructor and method wrappers
      cls.matlab_constructors(toolboxPath,nameSpace);
      cls.matlab_methods(classPath,nameSpace);

      // add lines to make m-file
      ofs << "cd(toolboxpath)" << endl;
      cls.matlab_make_fragment(ofs, toolboxPath, mexFlags);
    }  

    // finish make m-file
    ofs << "cd(toolboxpath)" << endl << endl;
    ofs << "echo off" << endl;
    ofs.close();
  }
  catch(exception &e) {
    cerr << "generate_matlab_toolbox failed because " << e.what() << endl;
  }

}

/* ************************************************************************* */
