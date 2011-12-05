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
 **/

#include "Module.h"
#include "utilities.h"

//#define BOOST_SPIRIT_DEBUG
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace wrap;
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
	       const string& moduleName, bool verbose) : name(moduleName), verbose_(verbose)
{
  // these variables will be imperatively updated to gradually build [cls]
  // The one with postfix 0 are used to reset the variables after parse.
	ReturnValue retVal0, retVal;
  Argument arg0, arg;
  ArgumentList args0, args;
  Constructor constructor0(verbose), constructor(verbose);
  Method method0(verbose), method(verbose);
  StaticMethod static_method0(verbose), static_method(verbose);
  Class cls0(verbose),cls(verbose);

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

  Rule comments_p =  comment_p("/*", "*/") |	comment_p("//");

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

  Rule eigenType =
    (str_p("Vector") | "Matrix")[assign_a(arg.type)] >>
    !ch_p('*')[assign_a(arg.is_ptr,true)];

  Rule name_p = lexeme_d[alpha_p >> *(alnum_p | '_')];

  Rule argument_p = 
    ((basisType_p[assign_a(arg.type)] | eigenType | classPtr_p | classRef_p) >> name_p[assign_a(arg.name)])
    [push_back_a(args, arg)]
    [assign_a(arg,arg0)];

  Rule argumentList_p = !argument_p >> * (',' >> argument_p);

  Rule constructor_p = 
    (className_p >> '(' >> argumentList_p >> ')' >> ';' >> !comments_p)
    [assign_a(constructor.args,args)]
    [assign_a(args,args0)]
    [push_back_a(cls.constructors, constructor)]
    [assign_a(constructor,constructor0)];

  Rule returnType1_p =
    basisType_p[assign_a(retVal.returns_)] |
    ((className_p | "Vector" | "Matrix")[assign_a(retVal.returns_)] >>
     !ch_p('*')  [assign_a(retVal.returns_ptr_,true)]);

  Rule returnType2_p =
    basisType_p[assign_a(retVal.returns2_)] |
    ((className_p | "Vector" | "Matrix")[assign_a(retVal.returns2_)] >>
     !ch_p('*')  [assign_a(retVal.returns_ptr2_,true)]);

  Rule pair_p = 
    (str_p("pair") >> '<' >> returnType1_p >> ',' >> returnType2_p >> '>')
    [assign_a(retVal.returns_pair_,true)];

  Rule void_p = str_p("void")[assign_a(retVal.returns_)];

  Rule returnType_p = void_p | returnType1_p | pair_p;

  Rule methodName_p = lexeme_d[lower_p >> *(alnum_p | '_')];

  Rule method_p = 
    (returnType_p >> methodName_p[assign_a(method.name_)] >>
     '(' >> argumentList_p >> ')' >> 
     !str_p("const")[assign_a(method.is_const_,true)] >> ';' >> *comments_p)
    [assign_a(method.args_,args)]
    [assign_a(args,args0)]
    [assign_a(method.returnVal_,retVal)]
    [assign_a(retVal,retVal0)]
    [push_back_a(cls.methods, method)]
    [assign_a(method,method0)];

  Rule staticMethodName_p = lexeme_d[upper_p >> *(alnum_p | '_')];

  Rule static_method_p =
    (str_p("static") >> returnType_p >> staticMethodName_p[assign_a(static_method.name_)] >>
     '(' >> argumentList_p >> ')' >> ';' >> *comments_p)
    [assign_a(static_method.args_,args)]
    [assign_a(args,args0)]
    [assign_a(static_method.returnVal_,retVal)]
    [assign_a(retVal,retVal0)]
    [push_back_a(cls.static_methods, static_method)]
    [assign_a(static_method,static_method0)];

  Rule methods_p = method_p | static_method_p;

  Rule class_p = str_p("class") >> className_p[assign_a(cls.name)] >> '{' >>
 		*comments_p >>
    *constructor_p >>
 		*comments_p >>
    *methods_p >>
 		*comments_p >>
    '}' >> ";";

  Rule module_p = *comments_p >> +(class_p
    [push_back_a(classes,cls)]
    [assign_a(cls,cls0)]
     >> *comments_p)
     >> *comments_p
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
  string contents = wrap::file_contents(interfaceFile);

  // and parse contents
  parse_info<const char*> info = parse(contents.c_str(), module_p, space_p);
  if(!info.full) {
    printf("parsing stopped at \n%.20s\n",info.stop);
    throw ParseFailed(info.length);
  }
}

/* ************************************************************************* */
void Module::matlab_code(const string& toolboxPath, 
			 const string& nameSpace, 
			 const string& mexExt,
			 const string& mexFlags)
{
  try {
    string installCmd = "install -d " + toolboxPath;
    system(installCmd.c_str());

    // create make m-file
    string matlabMakeFile = toolboxPath + "/make_" + name + ".m";
    ofstream ofs(matlabMakeFile.c_str());
    if(!ofs) throw CantOpenFile(matlabMakeFile);

    // create the (actual) make file
    string makeFile = toolboxPath + "/Makefile";
    ofstream make_ofs(makeFile.c_str());
    if(!make_ofs) throw CantOpenFile(makeFile);

    if (verbose_) cerr << "generating " << matlabMakeFile << endl;
    emit_header_comment(ofs,"%");
    ofs << "echo on" << endl << endl;
    ofs << "toolboxpath = mfilename('fullpath');" << endl;
    ofs << "delims = find(toolboxpath == '/');" << endl;
    ofs << "toolboxpath = toolboxpath(1:(delims(end)-1));" << endl;
    ofs << "clear delims" << endl;
    ofs << "addpath(toolboxpath);" << endl << endl;

    if (verbose_) cerr << "generating " << makeFile << endl;
    emit_header_comment(make_ofs,"#");
    make_ofs << "\nMEX = mex\n";
    make_ofs << "MEXENDING = " << mexExt << "\n";
    make_ofs << "mex_flags = " << mexFlags << "\n\n";

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
      cls.matlab_static_methods(toolboxPath,nameSpace);
      cls.matlab_methods(classPath,nameSpace);

      // add lines to make m-file
      ofs << "%% " << cls.name << endl;
      ofs << "cd(toolboxpath)" << endl;
      cls.matlab_make_fragment(ofs, toolboxPath, mexFlags);

      // add section to the (actual) make file
      make_ofs << "# " << cls.name << endl;
      cls.makefile_fragment(make_ofs);
    }  

    // finish make m-file
    ofs << "cd(toolboxpath)" << endl << endl;
    ofs << "echo off" << endl;
    ofs.close();

    // add 'all' and 'clean' to Makefile
    make_ofs << "\nall: ";
    BOOST_FOREACH(Class cls, classes)
    	make_ofs << cls.name << " ";

    make_ofs << "\n\nclean: \n";
    make_ofs << "\trm -rf *.$(MEXENDING)\n";
    BOOST_FOREACH(Class cls, classes)
    	make_ofs << "\trm -rf @" << cls.name << "/*.$(MEXENDING)\n";

    // finish Makefile
    make_ofs << "\n" << endl;
    make_ofs.close();
  }
  catch(exception &e) {
    cerr << "generate_matlab_toolbox failed because " << e.what() << endl;
  }

}

/* ************************************************************************* */
