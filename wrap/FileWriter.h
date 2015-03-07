/**
 * @file FileWriter.h
 *
 * @brief Wrapper for writing files and avoiding overwriting existing files
 * This class wraps a stream object and will check that the file is
 * actually different to write the new generated file.
 *
 * @date Jan 15, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <sstream>

namespace wrap {

class FileWriter {
protected:
	bool verbose_;
	std::string filename_;
	std::string comment_str_;

public:
	std::ostringstream oss; ///< Primary stream for operating on the file

	/** Create a writer with a filename and delimiter for the header comment */
	FileWriter(const std::string& filename, bool verbose, const std::string& comment_str);

	/** Writes the contents of the stringstream to the file, checking if actually new */
	void emit(bool add_header, bool force=false) const;

};

} // \namespace wrap
