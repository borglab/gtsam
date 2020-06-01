/**
 *  @file   FolderUtils.h
 *  @author Ryan
 *  @brief  Header file to allow for simple manipulation of directories.
 **/

#include <ctime>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>


using namespace std;
using namespace boost;

namespace gtsam {

//// function to get current timestamp
string getTimestamp();

/// construct new directory
void makeDir( string dir );

}
