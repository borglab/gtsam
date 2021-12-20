/**
 * @file GlobalFunction.h
 *
 * @brief Implements codegen for a global function wrapped in matlab
 *
 * @date Jul 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include "FullyOverloadedFunction.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace bl = boost::lambda;

namespace wrap {

struct GlobalFunction: public FullyOverloadedFunction {

  std::vector<Qualified> overloads; ///< Stack of qualified names
  std::string includeFile;

  // adds an overloaded version of this function,
  void addOverload(const Qualified& overload, const ArgumentList& args,
      const ReturnValue& retVal, const std::string& _includeFile = "", boost::optional<const Qualified> instName =
          boost::none, bool verbose = false);

  void verifyArguments(const std::vector<std::string>& validArgs) const {
    SignatureOverloads::verifyArguments(validArgs, name_);
  }

  void verifyReturnTypes(const std::vector<std::string>& validtypes) const {
    SignatureOverloads::verifyReturnTypes(validtypes, name_);
  }

  // codegen function called from Module to build the cpp and matlab versions of the function
  void matlab_proxy(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

  // emit python wrapper
  void python_wrapper(FileWriter& wrapperFile) const;

  // function name in Cython pxd
  std::string pxdName() const { return "pxd_" + pyRename(name_); }
  // function name in Python pyx
  std::string pyxName() const {
    std::string result = "";
    for(size_t i=0; i<overloads[0].namespaces_.size(); i++){
      if (i >= 1) {
        result += (overloads[0].namespaces_[i] + "_");
      }
    }
    result += pyRename(name_);
    return result;
  }

  // emit cython wrapper
  void emit_cython_pxd(FileWriter& pxdFile) const;
  void emit_cython_pyx(FileWriter& pyxFile) const;
  void emit_cython_pyx_no_overload(FileWriter& pyxFile) const;

private:

  // Creates a single global function - all in same namespace
  void generateSingleFunction(const std::string& toolboxPath,
      const std::string& wrapperName, const TypeAttributesTable& typeAttributes,
      FileWriter& file, std::vector<std::string>& functionNames) const;

};

typedef std::map<std::string, GlobalFunction> GlobalFunctions;

/* ************************************************************************* */
// http://boost-spirit.com/distrib/spirit_1_8_2/libs/spirit/doc/grammar.html
struct GlobalFunctionGrammar: public classic::grammar<GlobalFunctionGrammar> {

  GlobalFunctions& global_functions_; ///< successful parse will be placed in here
  std::vector<std::string>& namespaces_;
  std::string& includeFile;

  /// Construct type grammar and specify where result is placed
  GlobalFunctionGrammar(GlobalFunctions& global_functions,
                        std::vector<std::string>& namespaces,
                        std::string& includeFile)
      : global_functions_(global_functions),
        namespaces_(namespaces),
        includeFile(includeFile) {}

  /// Definition of type grammar
  template<typename ScannerT>
  struct definition: BasicRules<ScannerT> {

//    using BasicRules<ScannerT>::name_p;
//    using BasicRules<ScannerT>::className_p;
    using BasicRules<ScannerT>::comments_p;

    ArgumentList args;
    ArgumentListGrammar argumentList_g;

    ReturnValue retVal0, retVal;
    ReturnValueGrammar returnValue_g;

    Qualified globalFunction;

    classic::rule<ScannerT> globalFunctionName_p, global_function_p;

    definition(GlobalFunctionGrammar const& self) :
        argumentList_g(args), returnValue_g(retVal) {

      using namespace classic;
      bool verbose = false; // TODO

      globalFunctionName_p = lexeme_d[(upper_p | lower_p) >> *(alnum_p | '_')];

      // parse a global function
      global_function_p = (returnValue_g >> globalFunctionName_p[assign_a(
                                                globalFunction.name_)] >>
                           argumentList_g >> ';' >> *comments_p)    //
          [assign_a(globalFunction.namespaces_, self.namespaces_)]  //
          [bl::bind(
              &GlobalFunction::addOverload,
              bl::var(self.global_functions_)[bl::var(globalFunction.name_)],
              bl::var(globalFunction), bl::var(args), bl::var(retVal), bl::var(self.includeFile),
              boost::none, verbose)]  //
          [assign_a(retVal, retVal0)][clear_a(globalFunction)][clear_a(args)];
    }

    classic::rule<ScannerT> const& start() const {
      return global_function_p;
    }

  };
};
// GlobalFunctionGrammar

}// \namespace wrap

