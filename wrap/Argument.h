/**
 * file: Argument.h
 * brief: arguments to constructors and methods
 * Author: Frank Dellaert
 **/

#pragma once

#include <string>
#include <list>

// Argument class
struct Argument {
  bool is_const, is_ref, is_ptr;
  std::string type;
  std::string name;
Argument() : is_const(false), is_ref(false), is_ptr(false) {}

  // MATLAB code generation:
  void matlab_unwrap(std::ofstream& ofs, 
		     const std::string& matlabName); // MATLAB to C++
};

// Argument list
struct ArgumentList : public std::list<Argument> {
  std::list<Argument> args;
  std::string types    ();
  std::string signature();
  std::string names    ();

  // MATLAB code generation:

  /**
   * emit code to unwrap arguments
   * @param ofs output stream
   * @param start initial index for input array, set to 1 for method
   */
  void matlab_unwrap(std::ofstream& ofs, int start=0); // MATLAB to C++
};

