#pragma ident "$Id$"



/**
 * @file FileHunter.hpp
 * Find all files matching a specification.
 */

#ifndef GPSTK_FILEHUNTER_HPP
#define GPSTK_FILEHUNTER_HPP

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






#include "FileSpec.hpp"

namespace gpstk
{
   /** @addtogroup filedirgroup */
      //@{

      /// The only exception that class FileHunter throws.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(FileHunterException, gpstk::Exception);

      /**
       * FileHunter is a class designed to find files matching specified
       * criteria.  The specified criteria is threefold:
       * 1.  The File Specification.  This is a path and/or file description
       *  which is the most important search criteria.  For instance, the
       *  specification for a Rinex Obs file might be:
       *    /archive/ADMS%3n/RINEXOBS/S%2n%t%3jA.%2y0
       *  where the '%#char' elements denote fields such as year, day, station
       *  number, and so on.  In this example, FileHunter is smart enough to
       *  search all station directories (ADMS401, ADMS402, etc) but it won't
       *  search any directories (or subdirectories) other than ones that
       *  match the initial specification.
       * 2. Start and end times.  If you simply call find(), FileHunter will
       *  return all files that match the specification.  You can specify
       *  CommonTimes to only return files whose dates (according to the file
       *  name) fall between those times.  Note that you can tell find() to
       *  return the list of files ascending, descending, or unsorted
       *  (unsorted will return them in disk order).
       * 3. Filters.  Filters can be used on any field that your file
       *  specification includes.  The most appropriate use of this is to
       *  filter for a certain set of stations, but it will allow you to
       *  filter ANY field (day, year, etc) that is in the file specification.
       *  
       * For more information about file specifications, see the comments in
       * the FileSpec class.  Also check the file FileSpecTest in the test
       * subdirectory below this one for examples.
       *
       * Limitations:  If a file specification has two or more of the same
       * field defined, only the first one is used.  This is only if they
       * repeat in a single directory or path.  For instance, this would be ok:
       * /ADMS%3n/FILE%3n but in this case: /ADMS%3n/%3j%3j  only the first
       * %3j would be used for the day field.
       * Also, wildcards cannot be used.  Sorry - it's a limitation of how
       * regular expressions are used.
       * Finally, if you specify a version field, all files matching will
       * be reurned.  You can either use extractField() on the returned files
       * to determine the latest version or select the version you want
       * returned by using setFilter().
       */
   class FileHunter
   {
   public:

         /// This describes how the files to be searched are chunked
         /// i.e. a DAY chunking means one file is written per day
      enum FileChunking
      {
         WEEK,
         DAY,
         HOUR,
         MINUTE
      };
      
         /// Pairs of FileSpecTypes and lists of strings for filtering
         /// the search results.
      typedef std::pair<FileSpec::FileSpecType, std::vector<std::string> >
         FilterPair;

         /**
          * Constructs a FileHunter using a file specification.  This
          * can be a full or relative path and/or a file specification.
          * See FileSpec for details on how to format a file specification.
          * @throw FileHunterException when there's a problem with the filespec
          */
      FileHunter(const std::string& filespec)
         throw(FileHunterException);

         /**
          * Constructs a FileHunter using a FileSpec.
          * @throw FileHunterException when there's a problem with the filespec
          */
      FileHunter(const FileSpec& filespec)
         throw(FileHunterException);

         /** 
          * Rather than building a new file hunter, this lets you change
          * the filespec you're searching for.
          * @throw FileHunterException when there's a problem with the filespec
          */
      FileHunter& newHunt(const std::string& filespec)
         throw(FileHunterException);

         /**
          * Changes the file spec you're searching for in FileHunter.
          * @throw FileHunterException when there's an error in the filespec
          */
      FileHunter& newHunt(const FileSpec& filespec)
         throw(FileHunterException)
         {return newHunt(filespec.getSpecString());}

         /** 
          * Filters FOR the strings in /a filter in the field specified
          * by /a fst when searching.
          * @param fst A FileSpecType you want to run a filter on.
          * @param filter a list of strings to search for.in the filter.
          * @throw FileHunterException when /a fst can't be found.
          */
      FileHunter& setFilter(const FileSpec::FileSpecType fst,
                                    const std::vector<std::string>& filter)
         throw(FileHunterException);

         /**
          * Does the searching for the files.  Set /a start and /a end
          * for specifying times of files (according to their file names)
          * for returning.
          * \warning This method is NOT MT-Safe.
          * @param start the start time to limit the search
          * @param end the end time to limit the search
          * @param fsst set to change the order the list is returned
          * @param chunk the type of file chunking to use to select files
          * @return a list of files matching the file specification,
          *   start and end times, and filters ordered according to fsst.
          *   This list can be empty if no files are found.
          * @throw FileHunterException when there's a problem searching.
          */
      std::vector<std::string> 
      find(const gpstk::CommonTime& start = gpstk::CommonTime::BEGINNING_OF_TIME,
           const gpstk::CommonTime& end = gpstk::CommonTime::END_OF_TIME,
           const FileSpec::FileSpecSortType fsst = FileSpec::ascending,
           enum FileChunking chunk = DAY) const
         throw(FileHunterException);

         /// outputs the file spec into a semi-readable format
      void dump(std::ostream& o) const;

   private:
         // disallow these
      FileHunter();
      FileHunter(const FileHunter& fh);
      FileHunter& operator=(const FileHunter& fh);

   protected:
         ///Shared code between the constructor and newHunt
      void init(const std::string& filespec)
         throw(FileHunterException);

         /** Performs a search for the given file spec on the given directory.
          * \warning This method is NOT MT-Safe.
          * @return a list of matching directories/files */
      std::vector<std::string> 
      searchHelper(const std::string& directory,
                   const FileSpec& fs) const
         throw(FileHunterException);

         /// If there is a filter set, this will find the filtered items
         /// and set fileList to contain only those.
      void filterHelper(std::vector<std::string>& fileList, 
                        const FileSpec& fs) const
         throw(FileHunterException);

         /// Holds the broken down list of the file specification for searching
      std::vector<FileSpec> fileSpecList;

         /// Stores the list of things to filter for
      std::vector<FilterPair> filterList;
   }; // FileHunter

   //@}

} // namespace




#endif
