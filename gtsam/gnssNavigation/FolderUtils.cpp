/**
 * @file   FolderUtils.cpp
 * @brief  Tools to allow for simple manipulation of directories.

 * @author Ryan Watson
 */

#include <gtsam/gnssNavigation/FolderUtils.h>

using namespace std;
using namespace boost;

namespace gtsam {

/// Grab current system time.
string getTimestamp() {
        auto now = std::time(nullptr);
        char buf[sizeof("YYYY-MM-DD  HH:MM:SS")];
        string timeDateStr =  std::string(buf,buf + std::strftime(buf,sizeof(buf),"%F  %T",std::gmtime(&now)));
        replace_first(timeDateStr, "  ", "_");
        replace_all(timeDateStr, ":","-");
        return timeDateStr;
}

/// Use boost to construct new directory.
void makeDir( string dir ) {
        boost::filesystem::create_directories(dir);
}

}
