#pragma ident "$Id$"



/**
 * @file FileFilterFrameWithHeader.hpp
 * Wrapper for gpstk::FileHunter and gpstk::FileFilter that also handles header data
 */

#ifndef GPSTK_FILEFILTERFRAMEWITHHEADER_HPP
#define GPSTK_FILEFILTERFRAMEWITHHEADER_HPP

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






#include "FileFilterFrame.hpp"

namespace gpstk
{
   /** @addtogroup filedirgroup */
   //@{

      /**
       * This is just like FileFilterFrame but it can also handle header
       * data.  The header data is stored in an internal list that can be
       * accessed with the *Header() methods below.  Certain classes (like
       * RINEX Obs and Met) have filter functions that can be used with
       * touch() to combine header data from various files. The merge utilities
       * in wonky use this ability, and other file types with header data
       * can benefit from using this class's ability to store and write
       * header data - see the RINEX and FIC GFW classes for more examples.
       *
       * When initializing, this uses the FileFilterFrame::init() to read
       * the FileData into the filter, then does a second pass with its own
       * init() function to read the headers from those files. This
       * is a little inefficient, but the goal of these classes was never
       * efficiency.
       */
   template <class FileStream, class FileData, class FileHeader>
   class FileFilterFrameWithHeader :
      public FileFilterFrame<FileStream, FileData>
   {
   public:
         /// Default constructor
      FileFilterFrameWithHeader(const gpstk::CommonTime& start = 
                                   gpstk::CommonTime::BEGINNING_OF_TIME,
                                 const gpstk::CommonTime& end = 
                                   gpstk::CommonTime::END_OF_TIME)
         throw(gpstk::Exception) :
            FileFilterFrame<FileStream, FileData>(start, end)
         {}

         /// Takes a list of files to open in lieu of day times
      FileFilterFrameWithHeader(const std::vector<std::string>& fileList,
                                 const gpstk::CommonTime& start = 
                                    gpstk::CommonTime::BEGINNING_OF_TIME,
                                 const gpstk::CommonTime& end = 
                                    gpstk::CommonTime::END_OF_TIME)
         throw(gpstk::Exception) :
            FileFilterFrame<FileStream, FileData>(fileList, start, end)
         {
            std::vector<std::string>::const_iterator itr = fileList.begin();
            while (itr != fileList.end())
            {
               this->fs.newSpec(*itr);
               init();
               itr++;
            }
         }

         /// Takes a file name for a single file filter.
         /// This can throw an exception when there's a file error.
      FileFilterFrameWithHeader(const std::string& filename, 
                                 const gpstk::CommonTime& start = 
                                    gpstk::CommonTime::BEGINNING_OF_TIME,
                                 const gpstk::CommonTime& end = 
                                    gpstk::CommonTime::END_OF_TIME)
         throw(gpstk::Exception) :
            FileFilterFrame<FileStream, FileData>(filename, start, end)
         {init();}

         /// Uses the FileSpec to retrieve files.  Use filter like you would
         /// in FileHunter, to filter FOR stations, receivers, etc.
         /// This can throw an exception when there's a file error.
      FileFilterFrameWithHeader(const FileSpec& spec, 
                                 const gpstk::CommonTime& start = 
                                    gpstk::CommonTime::BEGINNING_OF_TIME,
                                 const gpstk::CommonTime& end = 
                                    gpstk::CommonTime::END_OF_TIME,
                                 const std::vector<FileHunter::FilterPair>& filter = 
                                    std::vector<FileHunter::FilterPair>())
         throw(gpstk::Exception) :
            FileFilterFrame<FileStream, FileData>(spec, start, end, filter)
         {init(filter);}

         /// Gets the files from the file spec and the time, then adds
         /// the data to the filter. Use filter like you would
         /// in FileHunter, to filter FOR stations, receivers, etc.
      FileFilterFrameWithHeader& 
      newSource(const FileSpec& filespec, 
                const gpstk::CommonTime& start = 
                   gpstk::CommonTime::BEGINNING_OF_TIME,
                const gpstk::CommonTime& end = 
                   gpstk::CommonTime::END_OF_TIME,
                const std::vector<FileHunter::FilterPair>& filter = 
                   std::vector<FileHunter::FilterPair>())
         throw(gpstk::Exception)
         {
            FileFilterFrame<FileStream, FileData>::newSource(filespec, start,
                                                             end, filter);
            init(filter);
            return *this;
         }

         /// Reads in the file and adds the data to the filter.
      FileFilterFrameWithHeader& 
      newSource(const std::string& filename, 
                const gpstk::CommonTime& start = 
                   gpstk::CommonTime::BEGINNING_OF_TIME,
                const gpstk::CommonTime& end = 
                   gpstk::CommonTime::END_OF_TIME)
         throw(gpstk::Exception)
         {
            FileFilterFrame<FileStream, FileData>::newSource(filename, start,
                                                             end);
            init();
            return *this;
         }

         /// Reads in the file and adds the data to the filter.
      FileFilterFrameWithHeader& 
      newSource(const std::vector<std::string>& fileList, 
                const gpstk::CommonTime& start = 
                   gpstk::CommonTime::BEGINNING_OF_TIME,
                const gpstk::CommonTime& end = 
                   gpstk::CommonTime::END_OF_TIME)
         throw(gpstk::Exception)
         {
            FileFilterFrame<FileStream, FileData>::newSource(fileList, start,
                                                             end);
            typename std::vector<std::string>::const_iterator itr;
            for (itr = fileList.begin(); itr != fileList.end(); itr++)
            {
               this->fs.newSpec(*itr);
               init();
            }
            return *this;
         }

      virtual ~FileFilterFrameWithHeader() {}

         /**
          * Writes the data to the file outputFile with the given header.
          * This will overwrite any existing file with the same name.
          * This can throw an exception when there's a file error.
          * @return true when it works.
          * @warning This will not write out headers for files that need them.
          */
      bool writeFile(const std::string& outputFile,
                     const FileHeader& fh) const
         throw(gpstk::Exception);

         /** performs the operation op on the header list. */
      template <class Operation>
      FileFilterFrameWithHeader& touchHeader(Operation& op)
      {
         typename std::list<FileHeader>::iterator itr = headerList.begin();

         while (itr != headerList.end())
         {
            op(*itr);
            itr++;
         }

         return *this;
      }

         /// Returns the contents of the header data list.
      std::list<FileHeader>& getHeaderData(void) {return headerList;}

         /// Returns the contents of the header data list, const.
      std::list<FileHeader> getHeaderData(void) const {return headerList;}

         /// Returns the number of data items in the header list.
      typename std::list<FileHeader>::size_type getHeaderCount(void) const 
         { return headerList.size(); }

      typename std::list<FileHeader>::const_iterator beginHeader() const
         throw(gpstk::InvalidRequest);

      typename std::list<FileHeader>::const_iterator endHeader() const
         throw(gpstk::InvalidRequest);

      typename std::list<FileHeader>::iterator beginHeader() 
         throw(gpstk::InvalidRequest);

      typename std::list<FileHeader>::iterator endHeader() 
         throw(gpstk::InvalidRequest);

      bool emptyHeader() const
         { return headerList.empty(); }

      typename std::list<FileHeader>::size_type sizeHeader()
         { return headerList.size(); }

      FileHeader& frontHeader()
         throw(gpstk::InvalidRequest);

      const FileHeader& frontHeader() const
         throw(gpstk::InvalidRequest);

      FileHeader& backHeader()
         throw(gpstk::InvalidRequest);

      const FileHeader& backHeader() const
         throw(gpstk::InvalidRequest);

   protected:
         ///  Run init() to load the data into the filter.  
      void init(const std::vector<FileHunter::FilterPair>& filter= 
                std::vector<FileHunter::FilterPair>()) 
         throw(gpstk::Exception);

         /// Check to make sure headerList is empty
         /// @throw InvalidRequest if headerList is empty
      inline void chl(const std::string& req) throw(gpstk::InvalidRequest)
      {
         gpstk::InvalidRequest exc("Header list is empty attempting to"
                                    " satisfy "+req+" request.");
         if (headerList.empty())
         {
            GPSTK_THROW(exc);
         }
      }

   protected:   
      std::list<FileHeader> headerList;
   };

   //@}

   template <class FileStream, class FileData, class FileHeader>
   bool FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::
   writeFile(const std::string& outputFile,
             const FileHeader& fh) const
      throw(gpstk::Exception)
   {
         // make the directory (if needed)
      std::string::size_type pos = outputFile.rfind('/');
      if (pos != std::string::npos)
         gpstk::FileUtils::makeDir(outputFile.substr(0,pos).c_str(), 0755);
      
      FileStream stream(outputFile.c_str(), std::ios::out|std::ios::trunc);
      stream.exceptions(std::ios::failbit);

      stream << fh;

      typename std::list<FileData>::const_iterator index;
      for(index = this->dataVec.begin(); index != this->dataVec.end(); index++)
         stream << (*index);
      return true;
   }

    template <class FileStream, class FileData, class FileHeader>
   typename std::list<FileHeader>::const_iterator
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::beginHeader()
      const throw(gpstk::InvalidRequest)
   {
      try { chl("beginHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.begin();
   }

   template <class FileStream, class FileData, class FileHeader>
   typename std::list<FileHeader>::const_iterator
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::endHeader()
      const throw(gpstk::InvalidRequest)
   {
      try { chl("endHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.end();
   }

   template <class FileStream, class FileData, class FileHeader>
   typename std::list<FileHeader>::iterator
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::beginHeader()
      throw(gpstk::InvalidRequest)
   {
      try { chl("beginHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.begin();
   }


   template <class FileStream, class FileData, class FileHeader>
   typename std::list<FileHeader>::iterator
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::endHeader()
      throw(gpstk::InvalidRequest)
   {
      try { chl("endHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.end();
   }

   template <class FileStream, class FileData, class FileHeader>
   FileHeader&
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::frontHeader()
      throw(gpstk::InvalidRequest)
   {
      try { chl("frontHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.front();
   }


   template <class FileStream, class FileData, class FileHeader>
   const FileHeader&
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::frontHeader()
      const throw(gpstk::InvalidRequest)
   {
      try { chl("frontHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.front();
   }


   template <class FileStream, class FileData, class FileHeader>
   FileHeader&
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::backHeader()
      throw(gpstk::InvalidRequest)
   {
      try { chl("backHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.back();
   }


   template <class FileStream, class FileData, class FileHeader>
   const FileHeader&
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader>::backHeader()
      const throw(gpstk::InvalidRequest)
   {
      try { chl("backHeader"); }
      catch(gpstk::InvalidRequest exc)
      { GPSTK_RETHROW(exc); }
      return headerList.back();
   }

   template <class FileStream, class FileData, class FileHeader>
   void
   FileFilterFrameWithHeader<FileStream,FileData,FileHeader> :: 
   init(const std::vector<FileHunter::FilterPair>& filter)
      throw(gpstk::Exception)
   {
         // find the files
      FileHunter fh(this->fs);
   
      typename std::vector<FileHunter::FilterPair>::const_iterator itr = 
         filter.begin();
      while (itr != filter.end())
      {
         fh.setFilter((*itr).first, (*itr).second);
         itr++;
      }

      std::vector<std::string> listOfFiles = 
         fh.find(this->startTime, this->endTime, FileSpec::ascending);

         // for each file, just read the header
      typename std::vector<std::string>::iterator i;
      for(i = listOfFiles.begin(); i != listOfFiles.end(); i++)
      {
         FileStream s((*i).c_str());

         if (s.good())
         {
            s.exceptions(std::ios::failbit);

            FileHeader header;
            s >> header;
            headerList.push_back(header);
         }
      }
   }  

} // namespace gpstk

#endif //GPSTK_FILEFILTERFRAMEWITHHEADER_HPP
