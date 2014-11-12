/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
 */

#include <boost/foreach.hpp>

#include "ReturnValue.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string ReturnValue::return_type(bool add_ptr, pairing p) const {
  if (p == pair && isPair) {
    string str = "pair< "
        + maybe_shared_ptr(add_ptr || isPtr1, qualifiedType1("::"), type1.name)
        + ", "
        + maybe_shared_ptr(add_ptr || isPtr2, qualifiedType2("::"), type2.name)
        + " >";
    return str;
  } else
    return maybe_shared_ptr(add_ptr && isPtr1,
        (p == arg2) ? qualifiedType2("::") : qualifiedType1("::"),
        (p == arg2) ? type2.name : type1.name);
}

/* ************************************************************************* */
string ReturnValue::matlab_returnType() const {
  return isPair? "[first,second]" : "result";
}

/* ************************************************************************* */
string ReturnValue::qualifiedType1(const string& delim) const {
  return type1.qualifiedName(delim);
}

/* ************************************************************************* */
string ReturnValue::qualifiedType2(const string& delim) const {
  return type2.qualifiedName(delim);
}

/* ************************************************************************* */
//TODO:Fix this
void ReturnValue::wrap_result(const string& result, FileWriter& file, const TypeAttributesTable& typeAttributes) const {
  string cppType1 = qualifiedType1("::"), matlabType1 = qualifiedType1(".");
  string cppType2 = qualifiedType2("::"), matlabType2 = qualifiedType2(".");

  if (isPair) {
    // For a pair, store the returned pair so we do not evaluate the function twice
    file.oss << "  " << return_type(false, pair) << " pairResult = " << result << ";\n";
    
    // first return value in pair
    if (category1 == ReturnValue::CLASS) { // if we are going to make one
      string objCopy, ptrType;
      ptrType = "Shared" + type1.name;
      const bool isVirtual = typeAttributes.at(cppType1).isVirtual;
      if(isVirtual) {
        if(isPtr1)
          objCopy = "pairResult.first";
        else
          objCopy = "pairResult.first.clone()";
      } else {
        if(isPtr1)
          objCopy = "pairResult.first";
        else
          objCopy = ptrType + "(new " + cppType1 + "(pairResult.first))";
      }
      file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType1 << "\", " << (isVirtual ? "true" : "false") << ");\n";
    } else if(isPtr1) {
      file.oss << "  Shared" << type1.name <<"* ret = new Shared" << type1.name << "(pairResult.first);" << endl;
      file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\", false);\n";
    } else // if basis type
      file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(pairResult.first);\n";

    // second return value in pair
    if (category2 == ReturnValue::CLASS) { // if we are going to make one
      string objCopy, ptrType;
      ptrType = "Shared" + type2.name;
      const bool isVirtual = typeAttributes.at(cppType2).isVirtual;
      if(isVirtual) {
        if(isPtr2)
          objCopy = "pairResult.second";
        else
          objCopy = "pairResult.second.clone()";
      } else {
        if(isPtr2)
          objCopy = "pairResult.second";
        else
          objCopy = ptrType + "(new " + cppType2 + "(pairResult.second))";
      }
      file.oss << "  out[1] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType2 << "\", " << (isVirtual ? "true" : "false") << ");\n";
    } else if(isPtr2) {
      file.oss << "  Shared" << type2.name <<"* ret = new Shared" << type2.name << "(pairResult.second);" << endl;
      file.oss << "  out[1] = wrap_shared_ptr(ret,\"" << matlabType2 << "\");\n";
    } else
      file.oss << "  out[1] = wrap< " << return_type(true,arg2) << " >(pairResult.second);\n";
  } else { // Not a pair

    if (category1 == ReturnValue::CLASS) {
      string objCopy, ptrType;
      ptrType = "Shared" + type1.name;
      const bool isVirtual = typeAttributes.at(cppType1).isVirtual;
      if(isVirtual) {
        if(isPtr1)
          objCopy = result;
        else
          objCopy = result + ".clone()";
      } else {
        if(isPtr1)
          objCopy = result;
        else
          objCopy = ptrType + "(new " + cppType1 + "(" + result + "))";
      }
      file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType1 << "\", " << (isVirtual ? "true" : "false") << ");\n";
    } else if(isPtr1) {
      file.oss << "  Shared" << type1.name <<"* ret = new Shared" << type1.name << "(" << result << ");" << endl;
      file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
    } else if (matlabType1!="void")
      file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(" << result << ");\n";
  }
}

/* ************************************************************************* */
void ReturnValue::wrapTypeUnwrap(FileWriter& wrapperFile) const {
  if(isPair)
  {
    if(category1 == ReturnValue::CLASS)
      wrapperFile.oss << "  typedef boost::shared_ptr<"  << qualifiedType1("::")  << "> Shared" <<  type1.name << ";"<< endl;
    if(category2 == ReturnValue::CLASS)
      wrapperFile.oss << "  typedef boost::shared_ptr<"  << qualifiedType2("::")  << "> Shared" <<  type2.name << ";"<< endl;
  }
  else {
    if (category1 == ReturnValue::CLASS)
      wrapperFile.oss << "  typedef boost::shared_ptr<"  << qualifiedType1("::")  << "> Shared" <<  type1.name << ";"<< endl;
  }
}
/* ************************************************************************* */
void ReturnValue::emit_matlab(FileWriter& file) const {
  string output;
  if (isPair)
    file.oss << "[ varargout{1} varargout{2} ] = ";
  else if (category1 != ReturnValue::VOID)
    file.oss << "varargout{1} = ";
}
/* ************************************************************************* */


