#pragma ident "$Id$"

/**
* @file ConfDataWriter.cpp
* Class to write configuration data files.
*/


//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//
//  Wei Yan - Chinese Academy of Sciences  2009, 2010
//
//============================================================================


#include "ConfDataWriter.hpp"

using namespace std;

namespace gpstk
{

	   // Method to open a configuration data file to be written.
	void ConfDataWriter::open(const char* fn)
	{
		FFTextStream::open(fn, std::ios::out);

		writeHeader();
		
		return;
	}
      // Write a common header for all of the configuration data files
      // written by this class. 
	void ConfDataWriter::writeHeader()
	{
     /* 
	   writeCommentLine("This is a configuration file written by ConfDataWriter, and it ");
      writeCommentLine("can be read by ConfDataReader.");
		writeCommentLine("YAN Wei,Dec,8th 2009");
		writeCommentLine("Enjoy!");
		writeCommentLine("");
		writeSeparatorLine();
		*/
      
	}

      // Write a comment line start by '#'
	void ConfDataWriter::writeCommentLine(const string& comment)
	{
		formattedPutLine("# "+comment);
	}


      // Write a comment line as a separator line
      // @param s    char of the separator line
      // @param n    size of the separator line
	void ConfDataWriter::writeSeparatorLine(const string& s,
                                           const int&    n )
	{
		writeCommentLine(string(n,s[0]));
	}

      // Write several blank lines default write one line
	void ConfDataWriter::writeBlankLine(const int& n)
	{
		int nLine = (n < 1) ? 0 : n;
		for(int i = 0;i < nLine; i++) 
      {
         formattedPutLine("");
      }
	}
      
      /* Write a string variable with general format
      *
      * @param var          variable name
      * @param val          variable value
      * @param varComment   variable comment 
      * @param valComment   value comment
      */
	void ConfDataWriter::writeVariable(const string& var,
                                      const string& val,
                                      const string& varComment,
                                      const string& valComment)
	{
		string line=var;

      if(int(var.length())<variableWidth) 
         line = StringUtils::leftJustify(var,variableWidth);

		if(varComment.length()>0) line += " , " + varComment;
		
      line += " = " + val;
		
      if(valComment.length() > 0) line += " , " + valComment;
		
      formattedPutLine(line);
	}

      /* Write a double variable with general format
       *
       * @param var          variable name
       * @param val          variable value
       * @param varComment   variable comment 
       * @param valComment   value comment
       */
   void ConfDataWriter::writeVariable(const string& var,
                      const double& val,
                      const string& varComment,
                      const string& valComment)
   { 
      writeVariable(var,StringUtils::asString(val,valuePrecison),
                                                         varComment,valComment);
   }

      

	   /* Write a string variable list with general format
       *
       * @param var          variable name
       * @param valList      variable list values
       * @param n            size of the variable list
       * @param varComment   variable comment 
       * @param valComment   value comment
       */
	void ConfDataWriter::writeVariableList(const string& var,
                                          const string  valList[],
                                          const int&    n,
                                          const string& varComment,
                                          const string& valComment)
	{
		string line=var;

      if(int(var.length())<variableWidth) 
         line = StringUtils::leftJustify(var,variableWidth);

		if(varComment.length() > 0) line += " , " + varComment;

		line += " = ";
		
		for(int i=0;i<n;i++) line += valList[i] + " ";

		
		if(valComment.length() > 0) line += " , " + valComment;

		formattedPutLine(line);
	}
	

      /* Write a string variable list with general format
       *
       * @param var          variable name
       * @param valList      variable list values by std::vector
       * @param varComment   variable comment 
       * @param valComment   value comment
       */
	void ConfDataWriter::writeVariableList(const string&         var,
                                          vector<string> valList,
                                          const string&         varComment,
                                          const string&         valComment)
	{
		string line=var;

      if(int(var.length())<variableWidth) 
         line = StringUtils::leftJustify(var,variableWidth);

		if(varComment.length() > 0) line += " , " + varComment;

		line += " = ";

		for(vector<string>::const_iterator it = valList.begin();
         it != valList.end();
         ++it )
		{
			line += (*it) + " ";
		}


		if(valComment.length() > 0) line += " , " + valComment;

		formattedPutLine(line);
	}


      /* Write a int variable list with general format
       *
       * @param var          variable name
       * @param valList      variable list values
       * @param n            size of the variable list
       * @param varComment   variable comment 
       * @param valComment   value comment
       */
	void ConfDataWriter::writeVariableList(const string& var,
                                          const int     valList[],
                                          const int&    n,
                                          const string& varComment,
                                          const string& valComment )
	{
		vector<string> vals;
		for(int i = 0; i < n; i++)
      {
         vals.push_back(StringUtils::asString(valList[i]));
      }

		writeVariableList(var, vals, varComment, valComment);
	}

      /* Write a double variable list with general format
       *
       * @param var          variable name
       * @param valList      variable list values
       * @param n            size of the variable list
       * @param varComment   variable comment 
       * @param valComment   value comment
       */
	void ConfDataWriter::writeVariableList(const string& var,
                                          const double valList[],
                                          const int&    n,
                                          const string& varComment,
                                          const string& valComment)
	{
		vector<string> vals;
		for(int i = 0; i < n; i++) 
      {
         vals.push_back(StringUtils::asString(valList[i],valuePrecison));
      }

		writeVariableList(var, vals, varComment, valComment);
	}
	
	
      /* Write a new section with some comment
       *
       * @param name          name of the section to be written
       * @param comment       comment of the section to be written
       */
	void ConfDataWriter::writeSection(const string& name,
                                     const string& comment)
	{
      string commentCopy(comment);

		if(commentCopy.length() < 1)
      {
         commentCopy = "Configuration data for '" + name + "' section";
      }
		
      writeCommentLine(StringUtils::upperCase(commentCopy));

		writeSeparatorLine();
		
      formattedPutLine("[" + StringUtils::strip(name) + "]");

	}


      // Write a common tailer for all of the configuration data files
      // written by this class.
	void ConfDataWriter::writeEnd()
	{
		writeBlankLine();

		writeCommentLine("End Of the File");
		
      writeSeparatorLine();
	}


      // Write a string line to the file.
	void ConfDataWriter::formattedPutLine(const std::string& sline)
	{
      // to make sure the line is less than 255 
		(*this) << sline.substr(0,255) << endl;
	}

}  // End of 'namespace gpstk'
