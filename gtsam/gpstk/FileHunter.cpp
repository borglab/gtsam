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
 * @file FileHunter.cpp
 * Find all files matching a specification.
 */

#include "FileHunter.hpp"
#include "YDSTime.hpp"
#include "CivilTime.hpp"
#include "GPSWeekSecond.hpp"

using namespace std;
using namespace gpstk;
using namespace gpstk::StringUtils;

// headers for directory searching interface
#ifndef _WIN32
#include <unistd.h>
#include <dirent.h>
#else
#include <io.h>
#include <direct.h>
#define PATH_MAX _MAX_PATH
#endif

namespace gpstk
{
   FileHunter::FileHunter(const string& filespec)
      throw(FileHunterException)
   {
      try
      {
         init(filespec);
      }
      catch (FileHunterException& e)
      {
         GPSTK_RETHROW(e);
      }
   }

   FileHunter::FileHunter(const FileSpec& filespec)
      throw(FileHunterException)
   {
      try
      {
         init(filespec.getSpecString());
      }
      catch (FileHunterException& e)
      {
         GPSTK_RETHROW(e);
      }
   }

   FileHunter& FileHunter::newHunt(const string& filespec)
      throw(FileHunterException)
   {
      try
      {
         init(filespec);
      }
      catch (FileHunterException& e)
      {
         GPSTK_RETHROW(e);
      }
      return *this;
   }

   FileHunter& FileHunter::setFilter(const FileSpec::FileSpecType fst,
                                     const vector<string>& filter)
      throw(FileHunterException)
   {
         // try to find the field in the fileSpecList.
      vector<FileSpec>::iterator itr = fileSpecList.begin();
      while (itr != fileSpecList.end())
      {
         if ((*itr).hasField(fst))
            break;
         itr++;
      }
         // found the field - add the filter.
      if (itr != fileSpecList.end())
      {
         filterList.push_back(FilterPair(fst, filter));
         return *this;
      }
         // didn't find it - throw an exception
      else
      {
         FileHunterException fhe("The FileSpec does not have a field: " +
                                 FileSpec::convertFileSpecType(fst));
         return *this;
      }
   }

   vector<string> FileHunter::find(const CommonTime& start,
                                   const CommonTime& end,
                                   const FileSpec::FileSpecSortType fsst,
                                   enum FileChunking chunk) const
      throw(FileHunterException)
   {
      
      // stupidity check
      if (end < start)
      {
         FileHunterException fhe("The times are specified incorrectly");
         GPSTK_THROW(fhe);
      }

         // move the start time back to a boundary defined by the file
         // chunking
      CommonTime exStart;
      switch(chunk)
      {
         case WEEK:
            //exStart = CivilTime(static_cast<GPSWeekSecond>(start).week, 0.0, static_cast<CivilTime>(start).year);
            exStart = GPSWeekSecond(static_cast<GPSWeekSecond>(start).week, 0.0);
            break;
         case DAY:
            //exStart = CivilTime(static_cast<YDSTime>(start).year, static_cast<YDSTime>(start).doy, 0.0);
            exStart = YDSTime(static_cast<YDSTime>(start).year, static_cast<YDSTime>(start).doy, 0.0);
            break;
         case HOUR:
            exStart = CivilTime(static_cast<YDSTime>(start).year, static_cast<CivilTime>(start).month,
                              static_cast<CivilTime>(start).day, static_cast<CivilTime>(start).hour,
                              0, 0.0);
            break;
         case MINUTE:
            exStart = CivilTime(static_cast<YDSTime>(start).year, static_cast<CivilTime>(start).month,
                              static_cast<CivilTime>(start).day, static_cast<CivilTime>(start).hour,
                              static_cast<CivilTime>(start).minute, 0.0);
            break;
      }
      
      vector<string> toReturn;
         // seed the return vector with an empty string.  you'll see why later
      toReturn.push_back(string());

         // debug
         /*
      printf("FileHunter.fileSpecList.size() = %d\n",(int)fileSpecList.size());
      int dcount = 0;
      vector<FileSpec>::const_iterator ditr;
      for (ditr=fileSpecList.begin(); ditr!=fileSpecList.end(); ++ditr)
      {
         printf("%2d:''%s''\n",dcount,(*ditr).getSpecString().c_str());
         dcount++;
      }
      printf("END OF LIST.\n");
          */

      try
      {
         vector<FileSpec>::const_iterator itr = fileSpecList.begin();

#ifdef _WIN32
            // If Windows, we should seed it with the drive spec
         if (itr != fileSpecList.end())
         {
            toReturn[0] = (*itr).getSpecString() + string(1,'\\');
            itr++;
         }
#endif
#ifdef __CYGWIN__
            // If Cygwin AND the user is attempting to use DOS file paths,
            // need to see with the /cygdrive "head".
         if (itr != fileSpecList.end())
         { 
            toReturn[0] = string(1,slash) + (*itr).getSpecString();
            itr++;
         }
#endif
         
         while (itr != fileSpecList.end())
         {
            vector<string> toReturnTemp;
            
               // counting variables            vector<string>::size_type i,j;
            
            for(size_t i = 0; i < toReturn.size(); i++)
            {
                  // search for the next entries
                  
         //Debug
         //printf("In .find() before call to searchHelper()\n");
         //string temp = (*itr).createSearchString();
         //printf(" toReturn[%d]:'%s', spec:'%s'\n",
         //         i,toReturn[i].c_str(),temp.c_str());

               vector<string> newEntries = searchHelper(toReturn[i],*itr);
         //Debug
         /*         
         vector<string>::iterator itr1 = newEntries.begin();
         int j1 = 0;
         printf("In .find() after call to searchHelper\n");
         printf("  newEntries.size() = %d\n",(int)newEntries.size() ); 
         while (itr1 != newEntries.end())
         {
            printf("newEntries[%d],item %d,'%s'\n",i,j1,(*itr1).c_str());
            itr1++;
            j1++;
         }
         printf("In .find().  end of newEntries list\n");
         */
                  // after getting the potential entries, filter
                  // them based on the user criteria...
               filterHelper(newEntries, *itr);

         //Debug
         /*
         printf("In .find() after call to filterHelper\n");
         
         vector<string>::iterator itr2 = newEntries.begin();
         int j2 = 0;
         while (itr2 != newEntries.end())
         {
            printf("newEntries[%d],item %d,'%s'\n",i,j2,(*itr2).c_str());
            itr2++;
            j2++;
         }
         printf("In .find().  end of newEntries list\n");
         printf("newEntries.size() : %d.\n", (int) newEntries.size() ); 
         */
         
                  // for each new entry, check the time (if possible)
                  // then add it if it's in the correct time range.
                  // this is why we need to enter an empty string to 
                  // seed toReturn
               for(size_t j = 0; j < newEntries.size(); j++)
               {
                  try
                  {
                     CommonTime fileDT = (*itr).extractCommonTime(newEntries[j]);
                     if ( (fileDT >= exStart) && (fileDT <= end) )
                     {
#ifdef _WIN32
                        if (toReturn[i].empty())
                           toReturnTemp.push_back(newEntries[j]);
                        else
                        {
                           if ( toReturn[i][toReturn[i].size()-1]=='\\')
                              toReturnTemp.push_back(toReturn[i] + newEntries[j]);
                           else
                              toReturnTemp.push_back(toReturn[i] + string(1,'\\') + 
                                               newEntries[j]);
                        }
#else
                        toReturnTemp.push_back(toReturn[i] + string(1,slash) + 
                                               newEntries[j]);
#endif
                     }
                  }
                     // if you can't make a CommonTime, just add it - 
                     // most likely, this is a field that you can't
                     // make a CommonTime out of
                  catch (FileSpecException &e)
                  {
#ifdef _WIN32
                        if (toReturn[i].empty())
                           toReturnTemp.push_back(newEntries[j]);
                        else
                        {
                           if ( toReturn[i][toReturn[i].size()-1]=='\\')
                              toReturnTemp.push_back(toReturn[i] + newEntries[j]);
                           else
                              toReturnTemp.push_back(toReturn[i] + string(1,'\\') + 
                                            newEntries[j]);
                        }
#else
                        toReturnTemp.push_back(toReturn[i] + string(1,slash) + 
                                            newEntries[j]);
#endif
                  }
               }
            }

            
            toReturn = toReturnTemp;
            
               // Debug
         /*         
         vector<string>::iterator itr3 = toReturn.begin();
         int j3 = 0;
         printf("In .find() just above toReturn empty check.\n");
         while (itr3 != toReturn.end())
         {
            printf("toReturn[%d],'%s'\n",j3,(*itr3).c_str());
            itr3++;
            j3++;
         }
         printf("In .find().  end of list\n");
         */

               // if at any time toReturn is empty, then there are no matches
               // so just return
            if (toReturn.empty())
               return toReturn;

            itr++;
         }

            // sort the list by the file spec of the last field
         itr--;
         (*itr).sortList(toReturn, fsst);

         return toReturn;
      }
      catch(...)
      {
         return toReturn;
      }
   }

   void FileHunter::init(const string& filespec)
      throw(FileHunterException)
   {
         // debug
      try
      {
         fileSpecList.clear();
         filterList.clear();

         string fs(filespec);

            // If working under Cygwin, then the file specification will be
            // handled as if it's a system rooted in '/'.  HOWEVER, if the 
            // user provided a spec that starts with a drive letter or
            // provided a relative path, it needs to be modified to fit the
            //
            // /cygdrive/<drive letter>/path
            //
            // For example,
            //     c:\ ->  /cygdrive/c
            //     something -> <cwd>/something
            //     c:\foo -> /cygdrive/c/foo
            //
            // form.
#ifdef __CYGWIN__
         //printf(" Entering 'ifdef __CYGWIN__' branch.\n");
         char backSlash = '\\';
         string::size_type st;
         if (fs[1] == ':')
         {
            //printf("Cygwin 'if' branch.  fs = '%s'.  Size = %d\n",
            //      fs.c_str(),fs.size());
            char driveLetter = fs[0];
            
               // Change all '\' to '/'
            while ((st = fs.find( backSlash )) != fs.npos)
            {
               //printf(" st = %d, ",st);
               fs = fs.replace(st, 1, 1, slash );
            }
            //printf(" end of back slash replacement.\n");
            //printf("After backslash replace.  fs = '%s'.\n",fs.c_str());
            
               // Remove drive letter and colon
            fs.erase(0,2);
            //printf("After removing draft letter and colon.  fs = '%s'.\n",fs.c_str());
            
               // Prepend "/cygdrive/driveLetter" to filespec
            fs.insert(0, 1, driveLetter);
            fs.insert(0,"/cygdrive/");
         }
         else
         {
            //printf("Cygwin 'else' branch.  fs = '%s'.\n",fs.c_str());
               // Get current working directory.
            char* cwd = getcwd(NULL, PATH_MAX);

               // If strokes are in wrong directon, fix them
            while ((st = fs.find( backSlash )) != fs.npos)
            {
               //printf(" st = %d, ",st);
               fs = fs.replace(st, 1, 1, slash );
            }
            //printf("After backslash replace.  fs = '%s'.\n",fs.c_str());

               // Prepend cwd to filespec
            if (fs[0]!=slash)
            {
               fs.insert(0,string(1,slash));
               fs.insert(0,cwd);
            }
         }
         //printf(" Operating under Cygwin.  Filespec after modification:\n");
         //printf(" fs = %s.\n",fs.c_str());
#endif
         
            // first, check if the file spec has a leading '/'.  if not
            // prepend the current directory to it.
#ifndef _WIN32
         if (fs[0] != slash)
         {
//                                                     #ifdef _WIN32
//          char* cwd = _getcwd(NULL, PATH_MAX);
//                                                     #else
            char* cwd = getcwd(NULL, PATH_MAX);
//                                                     #endif
            if (cwd == NULL)
            {
               FileHunterException fhe("Cannot get working directory");
               GPSTK_THROW(fhe);
            }
            string wd(cwd);
               // append a trailing slash if needed
            if (wd[wd.size()-1] != slash)
               wd += std::string(1,slash);
            fs.insert(0, wd);
            free(cwd);
         }
            // Append a closing slash so the breakdown algorithm has a
            // means to terminate.
         if (fs[fs.size()-1] != '/') fs += std::string(1,'/');
#else
            // If Windows, then check for leading drive name.
            // If not leading drivename, then prepend current working directory.
         if (fs[1]!=':')
         {
            char* cwdW = _getcwd(NULL, PATH_MAX);
            if (cwdW == NULL)
            {
               FileHunterException fhe("Cannot get working directory");
               GPSTK_THROW(fhe);
            }
            string wdW(cwdW);
            
               // append a trailing slash if needed
            if (wdW[wdW.size()-1] != '\\')
               wdW += std::string(1,'\\');
            fs.insert(0, wdW);
            free(cwdW);
         }
            // Append a closing slash so the breakdown algorithm has a
            // means to terminate.
         if (fs[fs.size()-1] != '\\') fs += std::string(1,'\\');
#endif

            // break down the filespec directory by directory into the
            // storage vector
         while (!fs.empty())
         {
#ifndef _WIN32
            if (fs[0] != slash)
            {
               FileHunterException fhe("Unexpected character: " + 
                                       fs.substr(0,1));
               GPSTK_THROW(fhe);
            }
            else
               // erase the leading slash
               fs.erase(0,1);
               
            string::size_type slashpos = fs.find(slash);
            FileSpec tempfs(fs.substr(0, slashpos));

               // debug
            //printf("FileHunter.init():  fs, slashpos, tempfs = '%s', %d, '%s'.\n",
            //   fs.c_str(),(int)slashpos,tempfs.getSpecString().c_str());

            if (slashpos!=string::npos) fileSpecList.push_back(tempfs);
            fs.erase(0, slashpos);
#else       
               // for Windows erase the leading backslash, if present
            if (fs[0] == '\\') fs.erase(0,1);
            string::size_type slashpos;
            slashpos = fs.find('\\');
            FileSpec tempfs(fs.substr(0, slashpos));
            
            if (slashpos!=string::npos) fileSpecList.push_back(tempfs);
            fs.erase(0, slashpos);
#endif
         }
      }
      catch(FileHunterException &e)
      {
         GPSTK_RETHROW(e);
      }
      catch(FileSpecException &e)
      {
         FileHunterException fhe(e);
         fhe.addText("Error in the file spec");
         GPSTK_THROW(fhe);
      }
      catch(Exception &e)
      {
         FileHunterException fhe(e);
         GPSTK_THROW(fhe);
      }
      catch(std::exception &e)
      {
         FileHunterException fhe("std::exception caught: " + string(e.what()));
         GPSTK_THROW(fhe);
      }
      catch(...)
      {
         FileHunterException fhe("unknown exception caught");
         GPSTK_THROW(fhe);
      }
   } // init
   
   vector<string> FileHunter::searchHelper(const string& directory,
                                           const FileSpec& fs) const
      throw(FileHunterException)
   {
      try
      {
         vector<string> toReturn;

            // generate a search string
         string searchString = fs.createSearchString();
         
#ifndef _WIN32
            // open the dir
         DIR* theDir;

         //printf(" In searchHelper(). About to call opendir.\n"); 
            // The first clause is a special kludge for Cygwin
            // referencing DOS drive structures
         //if (searchString.compare("cygdrive")==0)
         //{
         //   std::string tempFS =  std::string(1,slash) + searchString;
         //   theDir = opendir(tempFS.c_str());
         //}
         //else
         if (directory.empty())
            theDir = opendir(std::string(1,slash).c_str());
         else
            theDir = opendir(directory.c_str());

         //printf(" In searchHelper().  Back from opendir call.\n");
         
         if (theDir == NULL)
         {
            FileHunterException fhe("Cannot open directory: " + directory);
            GPSTK_THROW(fhe);
         }
         
            // get each dir/file entry and compare it to the search string
         struct dirent* entry;
         
         while ( (entry = readdir(theDir)) != NULL)
         {
            string filename(entry->d_name);
            
               // DEBUG
            //printf("Testing '%s'\n",filename.c_str());
            
            if ((filename.length() == searchString.length()) &&
                (filename != ".") && (filename != "..") && 
                isLike(filename, searchString, '*', '+', '?'))
            {
               toReturn.push_back(filename);
            }
         }
            // use filespec for extra verification?
         
            // cleanup
         if (closedir(theDir) != 0)
         {
            FileHunterException fhe("Error closing directory: " + 
                                    directory);
            GPSTK_THROW(fhe);
         }
#endif
#ifdef _WIN32
            // say 'hi' to old school MS io
         char* cwd = _getcwd(NULL, PATH_MAX);
         _chdir(directory.c_str());
         
         struct _finddata_t c_file;
         long hFile;
         
         if ( (hFile = _findfirst( searchString.c_str(), &c_file )) != -1 )
         {
            std::string filename(c_file.name);
            if ((filename != ".") && (filename != ".."))
            {
               toReturn.push_back(filename);
            }
            while( _findnext( hFile, &c_file ) == 0 )
            {
               filename = std::string(c_file.name);
               if ((filename != ".") && (filename != ".."))
               {
                  toReturn.push_back(filename);
               }
            }
         }
         _findclose(hFile);
         _chdir(cwd);
#endif
         return toReturn;
      }
      catch(Exception& e)
      {
         FileHunterException fhe(e);
         fhe.addText("Search failed");
         GPSTK_THROW(fhe);
      }
      catch(std::exception& e)
      {
         FileHunterException fhe("std::exception caught: " + string(e.what()));
         fhe.addText("Search failed");
         GPSTK_THROW(fhe);
      }
      catch(...)
      {
         FileHunterException fhe("unknown exception");
         fhe.addText("Search failed");
         GPSTK_THROW(fhe);         
      }
   }

   void FileHunter::filterHelper(vector<std::string>& fileList, 
                                 const FileSpec& fs) const
      throw(FileHunterException)
   {
         // go through the filterList.  If the filespec has
         // any fields to filter, remove matches from fileList

         // for each element in the filter....
      vector<FilterPair>::const_iterator filterItr = filterList.begin();
      while(filterItr != filterList.end())
      {
            // if the file spec has that element...
         if (fs.hasField((*filterItr).first))
         {

               // then search through the file list and 
               // remove any files that don't match the filter.
            vector<string>::iterator fileListItr = fileList.begin();
            while (fileListItr != fileList.end())
            {
                  // thisField holds the part of the file name
                  // that we're searching for
               string thisField = fs.extractField(*fileListItr, 
                                                  (*filterItr).first);
               
               vector<string>::const_iterator filterStringItr = 
                  (*filterItr).second.begin();

                  // the iterator searches each element of the filter
                  // and compares it to thisField.  If there's a match
                  // then keep it.  if there's no match, delete it.
               while (filterStringItr != (*filterItr).second.end())
               {
                  if (thisField == rightJustify(*filterStringItr,
                                               thisField.size(),
                                                '0'))
                     break;
                  filterStringItr++;
               }
               
               if (filterStringItr == (*filterItr).second.end())
                  fileList.erase(fileListItr);
               else
                  fileListItr++;
            }
         }
         filterItr++;
      }  
   }


   void FileHunter::dump(ostream& o) const
   {
      vector<FileSpec>::const_iterator itr = fileSpecList.begin();
      while(itr != fileSpecList.end())
      {
         (*itr).dump(o);
         itr++;
      }
   }

} // namespace


