#pragma ident "$Id$"

// expandtilde.hpp Expand tilde (~) in filenames.

#ifndef EXPAND_TILDE_INCLUDE
#define EXPAND_TILDE_INCLUDE

#include <string>
#include <vector>

void expand_filename(std::string& filename);
void expand_filename(std::vector<std::string>& sarray);
void include_path(std::string path, std::string& file);
void include_path(std::string path, std::vector<std::string>& sarray);
// return false if file cannot be opened
bool expand_list_file(std::string& filename, std::vector<std::string>& values);

#endif // EXPAND_TILDE_INCLUDE
