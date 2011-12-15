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
 **/

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include "Argument.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
void Argument::matlab_unwrap(ofstream& ofs, const string& matlabName) const {
  ofs << "  ";

  string cppType = qualifiedType("::");
  string matlabType = qualifiedType();

  if (is_ptr)
		// A pointer: emit an "unwrap_shared_ptr" call which returns a pointer
		ofs << "shared_ptr<" << cppType << "> " << name << " = unwrap_shared_ptr< ";
	else if (is_ref)
		// A reference: emit an "unwrap_shared_ptr" call and de-reference the pointer
		ofs << cppType << "& " << name << " = *unwrap_shared_ptr< ";
	else
		// Not a pointer or a reference: emit an "unwrap" call
		// unwrap is specified in matlab.h as a series of template specializations
		// that know how to unpack the expected MATLAB object
		// example: double tol = unwrap< double >(in[2]);
		// example: Vector v = unwrap< Vector >(in[1]);
		ofs << cppType << " " << name << " = unwrap< ";

	ofs << cppType << " >(" << matlabName;
  if (is_ptr || is_ref) ofs << ", \"" << matlabType << "\"";
  ofs << ");" << endl;
}

/* ************************************************************************* */
string Argument::qualifiedType(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces) result += ns + delim;
	return result + type;
}

/* ************************************************************************* */
string ArgumentList::types() const {
  string str;
  bool first=true;
  BOOST_FOREACH(Argument arg, *this)  {
    if (!first) str += ","; str += arg.type; first=false;
  }
  return str;
}

/* ************************************************************************* */
string ArgumentList::signature() const {
  string str;

  BOOST_FOREACH(Argument arg, *this)
  {
    BOOST_FOREACH(char ch, arg.type)
        if(isupper(ch))
            str += ch;

    if(str.length() == 0)
        str += arg.type[0];
  }

  return str;
}

/* ************************************************************************* */
string ArgumentList::names() const {
  string str;
  bool first=true;
  BOOST_FOREACH(Argument arg, *this) {
    if (!first) str += ","; str += arg.name; first=false;
  }
  return str;
}

/* ************************************************************************* */
void ArgumentList::matlab_unwrap(ofstream& ofs, int start) const {
  int index = start;
  BOOST_FOREACH(Argument arg, *this) {
    stringstream buf;
    buf << "in[" << index << "]";
    arg.matlab_unwrap(ofs,buf.str());
    index++;
  }
}

/* ************************************************************************* */
