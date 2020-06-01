#pragma ident "$Id$"



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
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================






/**
 * @file FileSpec.cpp
 * Define the specification of a file.
 */

#include <algorithm>

#include "FileSpec.hpp"
#include "TimeString.hpp"
#include "StringUtils.hpp"

using namespace std;
using namespace gpstk;
using namespace gpstk::StringUtils;

namespace gpstk
{
      // operator-- for FileSpecType
   FileSpec::FileSpecType& operator-- (FileSpec::FileSpecType& fst, int)
   {
      return fst = (fst == FileSpec::unknown) ? 
         FileSpec::end : FileSpec::FileSpecType(fst-1);
   }

      // operator++ for FileSpecType
   FileSpec::FileSpecType& operator++ (FileSpec::FileSpecType& fst, int)
   {
      return fst = (fst == FileSpec::end) ? 
         FileSpec::unknown : FileSpec::FileSpecType(fst+1);
   }

      // compares substrings of \a l and \a r
   bool FileSpec::FileSpecSort::operator() (const std::string& l, 
                                            const std::string& r) const
   {
         // if there are directories, don't include them in the comparison
      std::string tl(l, offset + l.find_last_of(slash) + 1, length);
      std::string tr(r, offset + r.find_last_of(slash) + 1, length);
      if (sortBy == ascending)
         return tl < tr;
      else
         return tl > tr;
   }
   
   string FileSpec::convertFileSpecType(const FileSpecType fst)
      throw(FileSpecException)
   {
      if (fst == station)          return string("n");
      else if (fst == receiver)    return string("r");
      else if (fst == prn)         return string("p");
      else if (fst == selected)    return string("t");
      else if (fst == sequence)    return string("I");
      else if (fst == version)     return string("v");
      else if (fst == fixed)       return string("");
      else if (fst == clock)       return string("k");
      else if (fst == text)        return string("x");

      else if (fst == year)        return string("y");
      else if (fst == month)       return string("m");
      else if (fst == dayofmonth)  return string("d");
      else if (fst == hour)        return string("H");
      else if (fst == minute)      return string("M");
      else if (fst == second)      return string("S");
      else if (fst == fsecond)     return string("f");
      else if (fst == gpsweek)     return string("G");
      else if (fst == fullgpsweek) return string("F");
      else if (fst == gpssecond)   return string("g");
      else if (fst == mjd)         return string("Q");
      else if (fst == dayofweek)   return string("w");
      else if (fst == day)         return string("j");
      else if (fst == doysecond)   return string("s");
      else if (fst == zcount)      return string("Z");
      else if (fst == zcountfloor) return string("z");
      else if (fst == unixsec)     return string("U");
      else if (fst == unixusec)    return string("u");
      else if (fst == fullzcount)  return string("C");
      else
      {
         FileSpecException fse("Unknown FileSpecType: " + asString(fst));
         GPSTK_THROW(fse);
      }
   }

   FileSpec::FileSpecType FileSpec::convertFileSpecType(const string& fst)
      throw(FileSpecException)
   {
      if (fst == string("n"))        return station;
      else if (fst == string("r"))   return receiver;
      else if (fst == string("p"))   return prn;
      else if (fst == string("t"))   return selected;
      else if (fst == string("I"))   return sequence;
      else if (fst == string("v"))   return version;
      else if (fst == string("k"))   return clock;
      else if (fst == string("x"))   return text;

      else if (fst == string("Y") || 
               fst == string("y"))   return year;
      else if (fst == string("m"))   return month;
      else if (fst == string("d"))   return dayofmonth;
      else if (fst == string("H"))   return hour;
      else if (fst == string("M"))   return minute;
      else if (fst == string("S"))   return second;
      else if (fst == string("f"))   return fsecond;
      else if (fst == string("G"))   return gpsweek;
      else if (fst == string("F"))   return fullgpsweek;
      else if (fst == string("g"))   return gpssecond;
      else if (fst == string("Q"))   return mjd;
      else if (fst == string("w"))   return dayofweek;
      else if (fst == string("j"))   return day;
      else if (fst == string("s"))   return doysecond;
      else if (fst == string("Z"))   return zcount;
      else if (fst == string("z"))   return zcountfloor;
      else if (fst == string("U"))   return unixsec;
      else if (fst == string("u"))   return unixusec;
      else if (fst == string("C") ||
               fst == string("c"))   return fullzcount;
      else
      {
         FileSpecException fse("Unknown FileSpecType: " + fst);
         GPSTK_THROW(fse);
      }
   }


   string FileSpec::createSearchString() const
      throw(FileSpecException)
   {
      string searchString;

         // go through the file spec element list...
      vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
      while (itr != fileSpecList.end())
      {
            // the error case first...
         if ( ((*itr).type <= unknown) || ((*itr).type >= end) )
         {
            FileSpecException fse("Unknown FileSpecType: " + 
                                  asString((*itr).type));
            GPSTK_THROW(fse);
         }
            // just add the fixed fields
         else if ((*itr).type == fixed)
         {
            searchString += (*itr).field;
         }
            // replace all the others with question marks for searching
         else
         {
            searchString += string((*itr).numCh, '?');
         }

         itr++;
      }

      return searchString;
   }

   string FileSpec::extractField(const string& filename, 
                                 const FileSpecType fst) const
      throw(FileSpecException)
   {
         // stupidity check - is it a valid FST?
      if ((fst <= unknown) || (fst >= end))
      {
         FileSpecException fse("Unknown FileSpecType: " + 
                               convertFileSpecType(fst));
         GPSTK_THROW(fse);
      }

         // check the FileSpec for this type of FST
      vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
      while (itr != fileSpecList.end())
      {
            // found it - get the substring and return
         if ((*itr).type == fst)
         {
            return filename.substr((*itr).offset, (*itr).numCh);
         }

            // didn't find it on this iteration
         itr++;
      }
         // oops - didn't find it.
      FileSpecException fse("Couldn't find specified FileSpecType: " +
                            convertFileSpecType(fst));
      GPSTK_THROW(fse);
   }

   bool FileSpec::hasField(const FileSpecType fst) const
      throw(FileSpecException)
   {
      vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
      while (itr != fileSpecList.end())
      {
            // stupidity check - is it a valid FST?
         if (((*itr).type <= unknown) || ((*itr).type >= end))
         {
            FileSpecException fse("Unknown FileSpecType: " + 
                                  convertFileSpecType((*itr).type));
            GPSTK_THROW(fse);
         }
         if ((*itr).type == fst)
            return true;
         itr++;
      }
      return false;
   }


   CommonTime FileSpec::extractCommonTime(const string& filename) const
      throw(FileSpecException)
   {
         // this uses CommonTime::setToString to get the time out
      try
      {
         CommonTime dt(0.L);
         mixedScanTime(dt, filename, fileSpecString);
         return dt;
      }
      catch(Exception& exc)
      {
            // too ambiguous - throw an exception
         FileSpecException fse(exc);
         fse.addText("Can't generate a CommonTime for this FileSpec");
         GPSTK_THROW(fse);
      }
      catch(std::exception& exc)
      {
         FileSpecException fse("std::exception: " + string(exc.what()));
         fse.addText("Can't generate a CommonTime for this FileSpec");
         GPSTK_THROW(fse);
      }
      catch(...)
      {
         FileSpecException fse("unknown exception");
         fse.addText("Can't generate a CommonTime for this FileSpec");
         GPSTK_THROW(fse);
      }
      
   }

   std::string FileSpec::toString(const gpstk::CommonTime& dt,
                                  const FSTStringMap& fstsMap) const
   {
      string toReturn;

         // Go through the list and insert all the non-date elements
         // into the string.  In other words, fill in the string with data
         // from the FSTSMap first.. For date elements, put the FileSpec string
         // directly into the file name (i.e. '%3j').  Then use CommonTime::printf
         // to fill in all the date elements at the end.
      vector<FileSpecElement>::const_iterator fslItr = fileSpecList.begin();
      while (fslItr != fileSpecList.end())
      {
         FSTStringMap::const_iterator fstsItr = fstsMap.find((*fslItr).type);
            // once again, it its found in the map, replace that part of
            // the file spec. otherwise, just put the fixed field in.
         if (fstsItr != fstsMap.end())
         {
               // special case for 'text': just print it
            if ((*fstsItr).first == text)
            {
               toReturn += (*fstsItr).second;
            }
            else
            {
               toReturn += 
                  rightJustify((*fstsItr).second, (*fslItr).numCh, '0');
            }
         }
         else
         {
            toReturn += (*fslItr).field;
         }

         fslItr++;
      }

      toReturn = printTime(dt,toReturn);

      return toReturn;
   }

   void FileSpec::sortList(vector<string>& fileList, 
                           const FileSpecSortType fsst) const
      throw(FileSpecException)
   {
         // gotta sort them in order as they appear in FileSpecType.
         // This is kinda like Radix sort... sort one field at a time.
      for(FileSpecType fst = FileSpecType(end-1); fst > unknown; fst--)
      {
         if (hasField(fst))
         {
               // check the FileSpec for this type of FST
            vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
            string::size_type ofs, len;
            while (itr != fileSpecList.end())
            {
                  // found it - get the substring and return
               if ((*itr).type == fst)
               {
                  ofs = (*itr).offset;
                  len = (*itr).numCh;
                  if (fsst != none)
                  {
                     FileSpecSort q(ofs, len, fsst);
                     stable_sort(fileList.begin(), fileList.end(), q);
                  }
               }
               
                  // didn't find it on this iteration
               itr++;
            }
         }
      }

         // to filter out versions, generate a list of the version FSEs first
         // and copy the file list.  then make a map of the file name without
         // the version field to the name with the version field. since its
         // sorted, the highest version will be the last one set and the map
         // will only have the latest versions...
         // 
         // ex.  a1a a2a a3a a4a a5a     file spec: a%1va
         // copyOfFileList after versions removed:  aa aa aa aa aa
         // versionMap[aa] = a1a then a2a, a3a, a4a, and finally a5a
         // 
         // note that this only handles 1 version field right now, not that
         // it couldnt do more but it gets very difficult...

         // filter out older versions here
      if (hasField(version))
      {
            // copy the file list
         vector<string> copyOfFileList = fileList;

            // find all the version elements in this file spec
         vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
         vector<FileSpecElement> versionVec;
         while (itr != fileSpecList.end())
         {
            if ((*itr).type == version)
               versionVec.push_back(*itr);
            itr++;
         }
            // remove the version fields from the copied list, but only for the
            // last directory/file entry in this name
         vector<string>::size_type index;
         for (index = 0; index < copyOfFileList.size(); index++)
         {
            string::size_type slashpos = copyOfFileList[index].rfind(slash);
            if (slashpos != string::npos)
               copyOfFileList[index].erase(0, slashpos + 1);
            copyOfFileList[index].erase(versionVec[0].offset,
                                        versionVec[0].numCh);
         }

            // now make one more pass on the copied list. whenever two strings
            // match, go to the original list and compare the version numbers.
            // erase the lower version. 

            // FIX: this will only compare the first version field encountered.
            // it could be changed to do more, but it's not essential now...
         map<string, string> versionMap;
         for (index = 0; index < copyOfFileList.size(); index++)
            versionMap[copyOfFileList[index]] = fileList[index];

         fileList.erase(fileList.begin(), fileList.end());
         map<string, string>::iterator mapitr = versionMap.begin();
         while (mapitr != versionMap.end())
         {
            fileList.push_back((*mapitr).second);
            mapitr++;
         }
      }
   }

   void FileSpec::dump(ostream& o) const
   {
      o << "FileSpec string: " << fileSpecString << endl;

      o << "offset numch  type  field" << endl;
      vector<FileSpecElement>::const_iterator itr = fileSpecList.begin();
      while (itr != fileSpecList.end())
      {
         o << setw(6) << (*itr).offset << setw(6) << (*itr).numCh
           << setw(6) << convertFileSpecType((*itr).type) 
               // this makes the field bigger if the string is
               // bigger than 5 characters
           << setw(6 + ((*itr).field.size() > 5 ? ((*itr).field.size()-5): 0))
           << (*itr).field << endl;
         itr++;
      }
   }

   void FileSpec::init(const string& fileSpec)
      throw(FileSpecException)
   {
      try
      {
         fileSpecList.clear();
         fileSpecString.clear();

         fileSpecString = fileSpec;

            // holds the offset for where we would be in the real file
            // name
         string::size_type offset = 0;

            // copy the string so we can mess with it
         string fs(fileSpec);
         
            // bit by bit, parse out the string into FileSpecElements,
            // stripping out the used parts as we go
         while (!fs.empty())
         {
            string atom;
               // if fs[0] == '%', then stop to parse.  also stop at
               // the end of the string
            string::size_type pos = fs.find('%');
            atom = fs.substr(0,pos);
            fs.erase(0,pos);

               // if it's at the end of the string...
               // make a FileSpecElement of any remaining
               // characters and return (fall through the while loop)
            if (fs.empty())
            {
               if (!atom.empty())
               {
                  FileSpecElement fse(atom.size(), offset, fixed, atom);
                  fileSpecList.push_back(fse);
               }
            }
               // found a '%' so parse out this little bit of a file spec,
               // but make sure to add atom to the FileSpec (if there is any)
            else
            {
               if (!atom.empty())
               {
                  FileSpecElement fse(atom.size(), offset, fixed, atom);
                  fileSpecList.push_back(fse);
                  offset += atom.size();
                  atom.erase(atom.begin(), atom.end());
               }
               
                  // erase the '%'
                  // also make sure that atom holds the string that
                  // makes up this element.
               atom += fs[0];
               fs.erase(0,1);
               
                  // get any integers that come before the letter we're lookin 
                  // for, then erase them
               int numChs = asInt(fs);
               if (numChs == 0)
                  numChs = 1;
               
               if (fs[0] == '0')
                  atom += '0';

               stripLeading(fs, "0");
               stripLeading(fs, asString(numChs));

               atom += asString(numChs);
               
                  // get the file spec type and erase that part of the string
               FileSpecType fst = convertFileSpecType(fs.substr(0,1));
               atom += fs[0];

                  // super special case - %Y -> %4y  FIX shouldn't this be <4?
               if ((fs.substr(0,1) == string("Y")) && (numChs != 4))
                  numChs = 4;
               fs.erase(0,1);
               
               FileSpecElement fse(numChs, offset, fst, atom);
               fileSpecList.push_back(fse);
               offset += numChs;
            }
            
         } // while !fs.empty()
      }
      catch(FileSpecException& e)
      {
         e.addText("Check your file spec for errors: " + fileSpec);
         GPSTK_RETHROW(e);
      }
      catch(StringException& e)
      {
         FileSpecException fse(e);
         fse.addText("String exception: Check the file spec for errors: " + fileSpec);
         GPSTK_THROW(fse);
      }
      catch(std::exception& e)
      {
         FileSpecException fse("std::exception: " + string(e.what()));
         fse.addText("Check the file spec for errors: " + fileSpec);
         GPSTK_THROW(fse);
      }
      catch(...)
      {
         FileSpecException fse("Unknown exception: Check the file spec for errors: " + fileSpec);
         GPSTK_THROW(fse);
      }
   }

} // namespace
