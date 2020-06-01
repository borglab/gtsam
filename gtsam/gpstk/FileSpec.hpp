#pragma ident "$Id$"



/**
 * @file FileSpec.hpp
 * Define the specification of a file.
 */

#ifndef GPSTK_FILESPEC_HPP
#define GPSTK_FILESPEC_HPP

#include <vector>
#include <functional>
#include <map>

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






#include "CommonTime.hpp"

#ifdef _WIN32
const char slash = '\\';
#else
const char slash = '/';
#endif

namespace gpstk
{
   /** @addtogroup filedirgroup */
   //@{

      /// This exception is thrown when there is a problem with
      /// handling file specifications.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(FileSpecException, gpstk::Exception);

      /**
       * A FileSpec is how file names are defined.  For instance,
       * ASM%4Y.%3j is the file spec for a SMODF file.  The format is similar
       * to what you would use in sprintf(), and specifically it uses the same
       * fields as CommonTime::printf().  For the most predictable results,
       * fully specify the field length (as in %4Y vs. %Y).  Check the
       * FileSpecType enum values for how characters map to fields. Any new
       * fields added should conform to CommonTime or already existing fields,
       * if there are any.
       */
   class FileSpec
   {
   public:
         /** This enum lists the different possible elements you can
          * have in a FileSpec.
          *
          * @note 
          * Whenever a format is added or removed from the CommonTime
          * class, it more than likely should also be added or removed
          * from the FileSpec class.  Additionally, the format
          * character must not conflict with any of the existing
          * format characters in CommonTime or FileSpec.
          */
      enum FileSpecType
      {
         unknown,       ///< Unknown type
         station,       ///< 'n' A field for station numbers
         receiver,      ///< 'r' A field for the receiver number
         prn,           ///< 'p' A field for PRN number
         selected,      ///< 't' A field for selected/unselected receiver
         sequence,      ///< 'I' A sequence number, as in part 1, part 2, etc..
         version,       ///< 'v' A version number, as in version 2 
                        ///<     is more recent than version 1
         fixed,         ///< A field for fixed characters
         clock,         ///< 'k' A field for the clock number
         text,          ///< 'x' A field for arbitrary text


            // see CommonTime for more information on the following elements
         year,          ///< 'y' or 'Y' A field for a year
         month,         ///< 'm' A field for month (numeric)
         dayofmonth,    ///< 'd' A field for day-of-month  
         hour,          ///< 'H' A field for hours (out of 24)
         minute,        ///< 'M' A field for minutes (out of 60)
         second,        ///< 'S' A field for seconds (out of 60)
         fsecond,       ///< 'f' A field for seconds (float)
         gpsweek,       ///< 'G' A field for 10 bit GPS weeks
         fullgpsweek,   ///< 'F' A field for full GPS weeks
         gpssecond,     ///< 'g' A field for GPS second-of-week
         mjd,           ///< 'Q' A field for Modified Julian Date
         dayofweek,     ///< 'w' A field for day-of-week (numeric)
         day,           ///< 'j' A field for day of year
         doysecond,     ///< 's' a field for second-of-day (float)
         zcount,        ///< 'Z' A field for GPS Z-count
         zcountfloor,   ///< 'z' A field for GPS Z-count rounded down
         unixsec,       ///< 'U' A field for UNIX seconds
         unixusec,      ///< 'u' A field for UNIX microseconds
         fullzcount,    ///< 'C' or 'c' A field for Full GPS Z-count

         end            ///< A place holder for the end of this list
      };

         /// A map from a FileSpecType to a string, used in the
         /// toString function.
      typedef std::map<FileSpecType, std::string> FSTStringMap;


         /// An enum for identifying the sort order of the list returned
         /// from FileHunter.
      enum FileSpecSortType
      {
         none,
         ascending,
         descending
      };

         /// Default constructor
      FileSpec() {}

         /// Constructor with a string to parse
      FileSpec(const std::string& fileSpec)
         throw(FileSpecException)
         {init(fileSpec);}

         /// Destructor
      virtual ~FileSpec() {}

         /// Reinitializes this FileSpec with the new string
      virtual FileSpec& newSpec(const std::string& fileSpec)
         throw(FileSpecException)
         {init(fileSpec); return *this;}

         /// Returns the string of the filespec
      virtual std::string getSpecString(void) const
         {return fileSpecString;}

         /**
          * Returns a string that can be used to search for files
          * matching this FileSpec.  Essentailly turns every non-fixed
          * field into strings of '?'.
          * @throw FileSpecException when there's an error in the FileSpec
          */
      virtual std::string createSearchString() const
         throw(FileSpecException);

         /**
          * Given a file name and a field, returns that field from the string.
          * Use hasField() first to see if the field exists in the FileSpec.
          * If multiple fields of FileSpecType are defined, only the first
          * is returned.
          * @throw FileSpecException when the FileSpecType doesn't exist
          *  in the FileSpec
          */
      virtual std::string extractField(const std::string& filename, 
                               const FileSpecType) const
         throw(FileSpecException);

         /**
          * Given a field type, returns true if the FileSpec has that field.
          * @throw FileSpecException when you pass in an invalid FileSpecType
          */
      virtual bool hasField(const FileSpecType) const
         throw(FileSpecException);

         /** 
          * If possible, returns a CommonTime object with the time the file
          * represents.  Since the time resolution only goes to days for
          * most file types, all times are set to midnight of that day.
          * @throw FileSpecException when a time can't be formed
          */
      virtual gpstk::CommonTime extractCommonTime(const std::string& filename) const
         throw(FileSpecException);

         /**
          * For the given FileSpec, fills in the fields with the given
          * information and returns a string of that file name.  Use the
          * FSTStringMap to provide all the other non-time data for the
          * string (i.e. FSTSMap[station] = "85408";).  Any unspecified
          * field will be filled with 0's.  The one side affect of this
          * is that you can only specify one of each field in the map,
          * but you're likely not going to need two different station
          * numbers in the file name.  If you want, you can put the CommonTime
          * information into the FSTSMap, but it's not necessary.
          * @return the new filename.
          */
      virtual std::string toString(const gpstk::CommonTime& dt,
                                   const FSTStringMap& fstsMap = FSTStringMap()) 
         const;

         /**
          * Sort the list of files ascending or 
          * descending.  The fields of the files are sorted in the order
          * that they're specified in the FileSpecType enum.
          * The list fileList is modified as a result
          * of this.  If the files in fileList have paths listed, then
          * only the file name (taken to be the word after the last '/') 
          * will be used in the comparison.  This function also filters
          * out older versions of files in the fileList.
          */
      virtual void sortList(std::vector<std::string>& fileList, 
                    const FileSpecSortType fsst = ascending) const
         throw(FileSpecException);

         /// semi-nicely print the FileSpec to the stream.
      virtual void dump(std::ostream& o) const;

   protected:
         /// Parses the string into the FileSpec object
      virtual void init(const std::string& fileSpec)
         throw(FileSpecException);

   public:
         /**
          * Converts the FileSpecType to a string it is identified with.
          * @throw FileSpecException when FileSpecType doesn't match
          *  any known types
          */
      static std::string convertFileSpecType(const FileSpecType)
         throw(FileSpecException);

         /**
          * Converts the string into its corresponding FileSpecType
          * @throw FileSpecException when FileSpecType doesn't match
          *  any known types
          */
      static FileSpecType convertFileSpecType(const std::string&)
         throw(FileSpecException);

   protected:
         /// This is an internal, private class of FileSpec that holds
         /// information for one individual file specification element.
      class FileSpecElement
      {
      public:
            /// This is the default constructor too.
         FileSpecElement(const std::string::size_type numChars = 0, 
                         const std::string::size_type offs = 0, 
                         const FileSpecType fst = unknown,
                         const std::string& fld = std::string())
               : numCh(numChars), offset(offs), type(fst), field(fld)
            {};
         
            /// The number of characters this field is in the file name.
         std::string::size_type numCh;
            /// The offset in the string where this field begins
         std::string::size_type offset;
            /// The type which this field corresponds to.
         FileSpecType type;
            /// For fixed strings, the characters it represents.
            /// For all other types, it's the original string for
            /// that field (i.e. '%03j').
         std::string field;
      };

         /// Another private, internal class for doing sorting
         /// by substring.
      struct FileSpecSort : 
         public std::binary_function<std::string,std::string,bool>
      {
      public:
         FileSpecSort(std::string::size_type o, std::string::size_type l,
                      const FileSpecSortType s)
               : offset(o), length(l), sortBy(s) {}
            /// Compares two strings based on the substrings defined by
            /// offset and length accounting for any directory names
            /// in the strings
         bool operator() (const std::string& l, const std::string& r) const;
      private:
            /// the offset of the substring to compare
         std::string::size_type offset;
            /// the length of the substring to compare
         std::string::size_type length;
            /// ascending, descending or none
         FileSpecSortType sortBy;
      };

         /// Holds all of the FileSpecElements for this FileSpec
      std::vector<FileSpecElement> fileSpecList;
         /// Holds the string that the fileSpecList was generated from
      std::string fileSpecString;
          
      
   }; // class FileSpec

      /// Operator-- for FileSpecType
   FileSpec::FileSpecType& operator-- (FileSpec::FileSpecType& fst, int);
      /// Operator++ for FileSpecType
   FileSpec::FileSpecType& operator++ (FileSpec::FileSpecType& fst, int);

   //@}

} // namespace gpstk

#endif 
