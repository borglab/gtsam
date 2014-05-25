/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Argument.ccp
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#include "Argument.h"

#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string Argument::matlabClass(const string& delim) const {
  string result;
  BOOST_FOREACH(const string& ns, namespaces)
    result += ns + delim;
  if (type == "string" || type == "unsigned char" || type == "char")
    return result + "char";
  if (type == "Vector" || type == "Matrix")
    return result + "double";
  if (type == "int" || type == "size_t")
    return result + "numeric";
  if (type == "bool")
    return result + "logical";
  return result + type;
}

/* ************************************************************************* */
bool Argument::isScalar() const {
  return (type == "bool" || type == "char" || type == "unsigned char"
      || type == "int" || type == "size_t" || type == "double");
}

/* ************************************************************************* */
void Argument::matlab_unwrap(FileWriter& file, const string& matlabName) const {
  file.oss << "  ";

  string cppType = qualifiedType("::");
  string matlabUniqueType = qualifiedType();

  if (is_ptr)
    // A pointer: emit an "unwrap_shared_ptr" call which returns a pointer
    file.oss << "boost::shared_ptr<" << cppType << "> " << name
        << " = unwrap_shared_ptr< ";
  else if (is_ref)
    // A reference: emit an "unwrap_shared_ptr" call and de-reference the pointer
    file.oss << cppType << "& " << name << " = *unwrap_shared_ptr< ";
  else
    // Not a pointer or a reference: emit an "unwrap" call
    // unwrap is specified in matlab.h as a series of template specializations
    // that know how to unpack the expected MATLAB object
    // example: double tol = unwrap< double >(in[2]);
    // example: Vector v = unwrap< Vector >(in[1]);
    file.oss << cppType << " " << name << " = unwrap< ";

  file.oss << cppType << " >(" << matlabName;
  if (is_ptr || is_ref)
    file.oss << ", \"ptr_" << matlabUniqueType << "\"";
  file.oss << ");" << endl;
}

/* ************************************************************************* */
string Argument::qualifiedType(const string& delim) const {
  string result;
  BOOST_FOREACH(const string& ns, namespaces)
    result += ns + delim;
  return result + type;
}

/* ************************************************************************* */
string ArgumentList::types() const {
  string str;
  bool first = true;
  BOOST_FOREACH(Argument arg, *this) {
    if (!first)
      str += ",";
    str += arg.type;
    first = false;
  }
  return str;
}

/* ************************************************************************* */
string ArgumentList::signature() const {
  string sig;
  bool cap = false;

  BOOST_FOREACH(Argument arg, *this) {
    BOOST_FOREACH(char ch, arg.type)
      if (isupper(ch)) {
        sig += ch;
        //If there is a capital letter, we don't want to read it below
        cap = true;
      }
    if (!cap)
      sig += arg.type[0];
    //Reset to default
    cap = false;
  }

  return sig;
}

/* ************************************************************************* */
string ArgumentList::names() const {
  string str;
  bool first = true;
  BOOST_FOREACH(Argument arg, *this) {
    if (!first)
      str += ",";
    str += arg.name;
    first = false;
  }
  return str;
}

/* ************************************************************************* */
bool ArgumentList::allScalar() const {
  BOOST_FOREACH(Argument arg, *this)
      if (!arg.isScalar()) return false;
  return true;
}

/* ************************************************************************* */
void ArgumentList::matlab_unwrap(FileWriter& file, int start) const {
  int index = start;
  BOOST_FOREACH(Argument arg, *this) {
    stringstream buf;
    buf << "in[" << index << "]";
    arg.matlab_unwrap(file, buf.str());
    index++;
  }
}

/* ************************************************************************* */
void ArgumentList::emit_prototype(FileWriter& file, const string& name) const {
  file.oss << name << "(";
  bool first = true;
  BOOST_FOREACH(Argument arg, *this) {
    if (!first)
      file.oss << ", ";
    file.oss << arg.type << " " << arg.name;
    first = false;
  }
  file.oss << ")";
}
/* ************************************************************************* */
void ArgumentList::emit_call(FileWriter& file, const ReturnValue& returnVal,
    const string& wrapperName, int id, bool staticMethod) const {
  returnVal.emit_matlab(file);
  file.oss << wrapperName << "(" << id;
  if (!staticMethod)
    file.oss << ", this";
  file.oss << ", varargin{:});\n";
}
/* ************************************************************************* */
void ArgumentList::emit_conditional_call(FileWriter& file,
    const ReturnValue& returnVal, const string& wrapperName, int id,
    bool staticMethod) const {
  // Check nr of arguments
  file.oss << "if length(varargin) == " << size();
  if (size() > 0)
    file.oss << " && ";
  // ...and their types
  bool first = true;
  for (size_t i = 0; i < size(); i++) {
    if (!first)
      file.oss << " && ";
    file.oss << "isa(varargin{" << i + 1 << "},'" << (*this)[i].matlabClass(".")
        << "')";
    first = false;
  }
  file.oss << "\n";

  // output call to C++ wrapper
  file.oss << "        ";
  emit_call(file, returnVal, wrapperName, id, staticMethod);
}
/* ************************************************************************* */

