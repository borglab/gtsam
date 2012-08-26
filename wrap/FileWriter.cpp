/**
 * @file FileWriter.cpp
 *
 * @date Jan 15, 2012
 * @author Alex Cunningham
 */

#include "FileWriter.h"
#include "utilities.h"

#include <fstream>
#include <iostream>

using namespace std;
using namespace wrap;

/* ************************************************************************* */
FileWriter::FileWriter(const string& filename, bool verbose, const string& comment_str)
: verbose_(verbose),filename_(filename), comment_str_(comment_str)
{
}

/* ************************************************************************* */
void FileWriter::emit(bool add_header, bool force_overwrite) const {
	if (verbose_) cerr << "generating " << filename_ << " ";
	// read in file if it exists
	string existing_contents;
	bool file_exists = true;
	try {
		existing_contents = file_contents(filename_.c_str(), add_header);
	} catch (CantOpenFile& e) {
		file_exists = false;
	}

	// Only write a file if it is new, an update, or overwrite is forced
	string new_contents = oss.str();
	if (force_overwrite || !file_exists || existing_contents != new_contents) {
		ofstream ofs(filename_.c_str(), ios::binary); // Binary to use LF line endings instead of CRLF
		if (!ofs) throw CantOpenFile(filename_);

		// dump in stringstream
		ofs << new_contents;
		ofs.close();
		if (verbose_) cerr << " ...complete" << endl;

		// Add small message whenever writing a new file and not running in full verbose mode
		if (!verbose_)
			cout << "wrap: generating " << filename_ << endl;
	} else {
		if (verbose_) cerr << " ...no update" << endl;
	}
}
/* ************************************************************************* */




