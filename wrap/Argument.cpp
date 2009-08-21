/**
 * file: Argument.ccp
 * Author: Frank Dellaert
 **/

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>

#include "Argument.h"

using namespace std;

/* ************************************************************************* */
void Argument::matlab_unwrap(ofstream& ofs, 
			     const string& matlabName) 
{ 
  // the templated unwrap function returns a pointer
  // example: double tol = unwrap< double >(in[2]);
  ofs << "  ";

  if (is_ptr)      
    ofs << "shared_ptr<" << type << "> " << name << " = unwrap_shared_ptr< ";
  else if (is_ref) 
    ofs <<                         type << "& " << name << " = *unwrap_shared_ptr< ";
  else
    ofs <<                         type << " "  << name << " = unwrap< ";

  ofs << type << " >(" << matlabName;
  if (is_ptr || is_ref) ofs << ", \"" << type << "\"";
  ofs << ");" << endl;
}

/* ************************************************************************* */
string ArgumentList::types() { 
  string str;
  bool first=true;
  BOOST_FOREACH(Argument arg, *this)  {
    if (!first) str += ","; str += arg.type; first=false;
  }
  return str;
}

/* ************************************************************************* */
string ArgumentList::signature() { 
  string str;
  BOOST_FOREACH(Argument arg, *this)
    str += arg.type[0];
  return str;
}

/* ************************************************************************* */
string ArgumentList::names() { 
  string str;
  bool first=true;
  BOOST_FOREACH(Argument arg, *this) {
    if (!first) str += ","; str += arg.name; first=false;
  }
  return str;
}

/* ************************************************************************* */
void ArgumentList::matlab_unwrap(ofstream& ofs, int start) { 
  int index = start;
  BOOST_FOREACH(Argument arg, *this) {
    stringstream buf;
    buf << "in[" << index << "]";
    arg.matlab_unwrap(ofs,buf.str());
    index++;
  }
}

/* ************************************************************************* */
