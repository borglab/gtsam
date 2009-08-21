/**
 * file: utilities.ccp
 * Author: Frank Dellaert
 **/

#pragma once

#include <exception>
#include <sstream>

class CantOpenFile : public std::exception {
 private:
  std::string filename_;
 public:
 CantOpenFile(const std::string& filename) : filename_(filename) {}
  ~CantOpenFile() throw() {}
  virtual const char* what() const throw() { 
    return ("Can't open file " + filename_).c_str(); 
  }
};

class ParseFailed : public std::exception {
 private:
  int length_;
 public:
 ParseFailed(int length) : length_(length) {}
  ~ParseFailed() throw() {}
  virtual const char* what() const throw() { 
    std::stringstream buf;
    buf << "Parse failed at character " << (length_+1);
    return buf.str().c_str(); 
  }
};

/**
 * read contents of a file into a std::string
 */
std::string file_contents(const std::string& filename, bool skipheader=false);

/**
 * Check whether two files are equal
 * By default, skips the first line of actual so header is not generated
 */
bool files_equal(const std::string& actual, const std::string& expected, bool skipheader=true);

/**
 * emit a header at the top of generated files
 */
void emit_header_comment(std::ofstream& ofs, const std::string& delimiter);
