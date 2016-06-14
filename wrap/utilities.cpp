/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file utilities.ccp
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#include <iostream>
#include <cstdlib>

#include <boost/filesystem.hpp>

#include "utilities.h"

namespace wrap {

using namespace std;

/* ************************************************************************* */
string file_contents(const string& filename, bool skipheader) {
  ifstream ifs(filename.c_str());
  if(!ifs) throw CantOpenFile(filename);

  // read file into stringstream
  stringstream ss;
  if (skipheader) ifs.ignore(256,'\n');
  ss << ifs.rdbuf();
  ifs.close();

  // return string
  return ss.str();
}

/* ************************************************************************* */
bool assert_equal(const string& expected, const string& actual) {
  if (expected == actual)
    return true;
  printf("Not equal:\n");
  cout << "expected: [" << expected << "]\n";
  cout << "actual: [" << actual << "]" << endl;
  return false;
}

/* ************************************************************************* */
bool assert_equal(const vector<string>& expected, const vector<string>& actual) {
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  vector<string>::const_iterator
    itExp = expected.begin(),
    itAct = actual.begin();
  if(match) {
    for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
      if (*itExp != *itAct) {
        match = false;
        break;
      }
    }
  }
  if(!match) {
    cout << "expected: " << endl;
    for(const vector<string>::value_type& a: expected) { cout << "["  << a << "] "; }
    cout << "\nactual: " << endl;
    for(const vector<string>::value_type& a: actual) { cout << "["  << a << "] "; }
    cout << endl;
    return false;
  }
  return true;

}

/* ************************************************************************* */
bool files_equal(const string& expected, const string& actual, bool skipheader) {
  try {
    string expected_contents = file_contents(expected, skipheader);
    string actual_contents   = file_contents(actual, skipheader);
    bool equal = actual_contents == expected_contents;
    if (!equal) {
      cerr << "<<< DIFF OUTPUT (if none, white-space differences only):\n";
      stringstream command;
      command << "diff --ignore-all-space " << expected << " " << actual << endl;
      if (system(command.str().c_str())<0)
        throw "command '" + command.str() + "' failed";
      cerr << ">>>\n";
    }
    return equal;
  }
  catch (const string& reason) {
    cerr << "expection: " << reason << endl;
    return false;
  }
  catch (CantOpenFile& e) {
    cerr << "file opening error: " << e.what() << endl;
    return false;
  }
  return true;
}

/* ************************************************************************* */
string maybe_shared_ptr(bool add, const string& qtype, const string& type) {
  string str = add? "Shared" : "";
  if (add) str += type; 
  else str += qtype;
  return str;
}

/* ************************************************************************* */
string qualifiedName(const string& separator, const vector<string>& names) {
  string result;
  if (!names.empty()) {
    for (size_t i = 0; i < names.size() - 1; ++i)
      result += (names[i] + separator);
    result += names.back();
  }
  return result;
}

/* ************************************************************************* */
void createNamespaceStructure(const std::vector<std::string>& namespaces,
    const std::string& toolboxPath) {
  using namespace boost::filesystem;
  path curPath = toolboxPath;
  for(const string& subdir: namespaces) {
//    curPath /= "+" + subdir; // original - resulted in valgrind error
    curPath = curPath / string(string("+") + subdir);
    if(!is_directory(curPath)) {
      if(exists("+" + subdir))
        throw OutputError("Need to write files to directory " + curPath.string() + ", which already exists as a file but is not a directory");
      else
        boost::filesystem::create_directory(curPath);
    }
  }
}

/* ************************************************************************* */

} // \namespace wrap
