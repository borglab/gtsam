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

#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
Argument Argument::expandTemplate(const TemplateSubstitution& ts) const {
  Argument instArg = *this;
  instArg.type = ts.tryToSubstitite(type);
  return instArg;
}

/* ************************************************************************* */
ArgumentList ArgumentList::expandTemplate(
    const TemplateSubstitution& ts) const {
  ArgumentList instArgList;
  for(const Argument& arg: *this) {
    Argument instArg = arg.expandTemplate(ts);
    instArgList.push_back(instArg);
  }
  return instArgList;
}

/* ************************************************************************* */
string Argument::matlabClass(const string& delim) const {
  string result;
  for(const string& ns: type.namespaces())
    result += ns + delim;
  if (type.name() == "string" || type.name() == "unsigned char"
      || type.name() == "char")
    return result + "char";
  if (type.name() == "Vector" || type.name() == "Matrix")
    return result + "double";
  if (type.name() == "int" || type.name() == "size_t")
    return result + "numeric";
  if (type.name() == "bool")
    return result + "logical";
  return result + type.name();
}

/* ************************************************************************* */
bool Argument::isScalar() const {
  return (type.name() == "bool" || type.name() == "char"
      || type.name() == "unsigned char" || type.name() == "int"
      || type.name() == "size_t" || type.name() == "double");
}

/* ************************************************************************* */
void Argument::matlab_unwrap(FileWriter& file, const string& matlabName) const {
  file.oss << "  ";

  string cppType = type.qualifiedName("::");
  string matlabUniqueType = type.qualifiedName();

  if (is_ptr && type.category != Qualified::EIGEN)
    // A pointer: emit an "unwrap_shared_ptr" call which returns a pointer
    file.oss << "boost::shared_ptr<" << cppType << "> " << name
        << " = unwrap_shared_ptr< ";
  else if (is_ref && type.category != Qualified::EIGEN)
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
  if( (is_ptr || is_ref) && type.category != Qualified::EIGEN)
    file.oss << ", \"ptr_" << matlabUniqueType << "\"";
  file.oss << ");" << endl;
}

/* ************************************************************************* */
void Argument::proxy_check(FileWriter& proxyFile, const string& s) const {
  proxyFile.oss << "isa(" << s << ",'" << matlabClass(".") << "')";
  if (type.name() == "Vector")
    proxyFile.oss << " && size(" << s << ",2)==1";
}

/* ************************************************************************* */
string ArgumentList::types() const {
  string str;
  bool first = true;
  for(Argument arg: *this) {
    if (!first)
      str += ",";
    str += arg.type.name();
    first = false;
  }
  return str;
}

/* ************************************************************************* */
string ArgumentList::signature() const {
  string sig;
  bool cap = false;

  for(Argument arg: *this) {
    for(char ch: arg.type.name())
      if (isupper(ch)) {
        sig += ch;
        //If there is a capital letter, we don't want to read it below
        cap = true;
      }
    if (!cap)
      sig += arg.type.name()[0];
    //Reset to default
    cap = false;
  }

  return sig;
}

/* ************************************************************************* */
string ArgumentList::names() const {
  string str;
  bool first = true;
  for(Argument arg: *this) {
    if (!first)
      str += ",";
    str += arg.name;
    first = false;
  }
  return str;
}

/* ************************************************************************* */
bool ArgumentList::allScalar() const {
  for(Argument arg: *this)
    if (!arg.isScalar())
      return false;
  return true;
}

/* ************************************************************************* */
void ArgumentList::matlab_unwrap(FileWriter& file, int start) const {
  int index = start;
  for(Argument arg: *this) {
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
  for(Argument arg: *this) {
    if (!first)
      file.oss << ", ";
    file.oss << arg.type.name() << " " << arg.name;
    first = false;
  }
  file.oss << ")";
}

/* ************************************************************************* */
void ArgumentList::proxy_check(FileWriter& proxyFile) const {
  // Check nr of arguments
  proxyFile.oss << "if length(varargin) == " << size();
  if (size() > 0)
    proxyFile.oss << " && ";
  // ...and their type.names
  bool first = true;
  for (size_t i = 0; i < size(); i++) {
    if (!first)
      proxyFile.oss << " && ";
    string s = "varargin{" + boost::lexical_cast<string>(i + 1) + "}";
    (*this)[i].proxy_check(proxyFile, s);
    first = false;
  }
  proxyFile.oss << "\n";
}

/* ************************************************************************* */

