#pragma ident "$Id$"



/**
 * @file RTFileFrame.hpp
 * Realtime file frame
 */

#ifndef GPSTK_RTFILEFRAME_HPP
#define GPSTK_RTFILEFRAME_HPP

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






#include <sys/types.h>
#include <sys/stat.h>
#include <iterator>
#include "FileSpec.hpp"
#include "FileFilter.hpp"

namespace gpstk
{
   /** @addtogroup filedirgroup */
   //@{

      /// specifying internal enums and classes of a template class can be 
      /// problematic, so these are declared outside RTFileFrame
   class RTFileFrameHelper
   {
   public:
      enum GetRecordMode
      {
         Dumb  = 0, ///< only tries to read once from current file
         Smart = 1  ///< if cannot read, rolls files or waits for data
      };
      
         /**
          * Whether additional file data can be read from the end of a file
          * or not.  
          */
      enum FileReadingMode
      {
         AppendedData = 0,     ///< read only appended data.
         FromTheBeginning = 1  ///< reread from the beginning every time.
      };
   };

      // forward declaration of the RTFileFrame class
   template <class FileStream, class FileData>
   class RTFileFrame;

      /**
       * An iterator class for the RTFileFrame.  Note how much it looks
       * like std::istream_iterator except for a couple of differences..
       */
   template <class FileStream, class FileData>
   class RTFileFrameIterator
   {
   public:
         /// default constructor
      RTFileFrameIterator() 
            : rtff(NULL)
         {}

         /// copy constructor
      RTFileFrameIterator(const RTFileFrameIterator& right)
            : rtff(right.rtff)
         {}

         /// destructor
      virtual ~RTFileFrameIterator() {}

         /// assignment operator
      RTFileFrameIterator& operator=(const RTFileFrameIterator& r)
         { 
            rtff = r.rtff; 
            return *this; 
         }

         /// prefix increment of internal iterator
      RTFileFrameIterator operator++()
         {
            rtff->getRecord();
            return *this;
         }

         /// postfix increment of internal iterator
      RTFileFrameIterator operator++(int)
         {
            RTFileFrameIterator<FileStream, FileData> temp = *this;
            read();
            return temp;
         }

         /// dereference object for current object
      const FileData& operator*() const
         {
            return rtff->lastData;
         }

         /// dereference pointer for current object
      const FileData* operator->() const
         {
            return &(rtff->lastData);
         }
      
         /// equality operator
      bool operator==(const RTFileFrameIterator& right) const
         {
            if (rtff == right.rtff)
            {
               if (rtff == NULL)
                  return true;
               
               if ((rtff->isOK == right.rtff->isOK) ||
                   (!rtff->isOK && (right.rtff->fileStream == rtff->fileStream)))
                  return true;
               return false;
            }
            else if ((rtff == NULL) && (right.rtff))
               return !right.rtff->isOK;
            else if ((right.rtff == NULL) && (rtff))
               return !rtff->isOK;
            else
               return false;
         }

         /// inequality operator
      bool operator!=(const RTFileFrameIterator& right) const
         { return !(*this == right);}

         /// give RTFileFrame access to the 
      friend class RTFileFrame<FileStream, FileData>;
      
   protected:
         /// gets the next record
      void read() 
         { rtff->getRecord(); }

   private:
         /// constructor for use by gpstk::RTFileFrame<FileStream, FileData>.
      RTFileFrameIterator(gpstk::RTFileFrame<FileStream, FileData>& r)
            : rtff(&r)
         { rtff->getRecord(); }

         /// gives access to the RTFileFrame being used
      gpstk::RTFileFrame<FileStream, FileData>* rtff;
   };

      /**
       * This class supports two iteration semantics.
       * @code
       * RTFileFrameIterator<SMODFStream, SMODFData> itr = rtf.begin();
       * while(itr != rtf.end())
       * {
       *       // do something
       *    cout << *itr << endl;
       *    itr++;
       * }
       * rtf.waitForData(10);
       * @endcode
       * and the legacy interface
       * @code
       * while(rtf.getRecord())
       * {
       *      // use data() to get the last read record
       *    cout << rtf.data() << endl;
       * }
       * rtf.waitForData(10);
       * @endcode
       * The waitForData() call will ready the internal file stream 
       * for the next read.  Remember that RTFileFrameIterator is a
       * past-the-end style iterator, so you must reset it to begin() when
       * it has finished otherwise you'll get no data. In Dumb mode, 
       * you will need to use the openNextDay() function to advance
       * to tne next day.  
       * When you reach the end of a data set, there are three ways
       * to restore the valid state of the RTFileFrame iteration:
       * @code rtf.waitForData(900) @endcode
       * This is the best option for either Smart or Dumb mode, ensuring
       * that the file gets reopened correctly.
       * @code rtf.openCurrentFile() @endcode
       * Reopens the current file, destroying the current read position.
       * In other words, don't use this in Smart mode.
       * @code rtf.openNextDay() @endcode
       * This will open the next day's file, obviously resetting
       * the read position for the stream.
       * @warning In Smart mode, you could potentially miss data when
       * you pass to the start of a new day if data gets written to the
       * file of the previous day.  
       * @warning DO NOT MIX THE TWO ITERATOR SEMANTICS. Doing so, you will
       * definately miss data.
       * 
       *
       */
   template <class FileStream, class FileData>
   class RTFileFrame : public RTFileFrameHelper
   {
   public:
         /**
          * constructor
          * @param fnFormat string defining how the files names are formatted
          * @param beginning time of data to start streaming from
          * @param ending time of data to stop streaming at
          * @param frm mode used to read file (new data or all)
          * @param grm mode used when end of file is reached (new data or all)
          * @throw Exception an error ocurred
          */
      RTFileFrame(const gpstk::FileSpec& fnFormat,
                  const gpstk::CommonTime& beginning = gpstk::CommonTime::BEGINNING_OF_TIME,
                  const gpstk::CommonTime& ending = gpstk::CommonTime::END_OF_TIME, 
                  const FileReadingMode frm = AppendedData,
                  const GetRecordMode grm = Dumb)
         throw(gpstk::Exception);
      
         /// destructor
      ~RTFileFrame();
      
         /// Allows changing of the FileReadingMode
      RTFileFrame& setFileReadingMode(const FileReadingMode f);

         /// Allows changing of the GetRecordMode
      RTFileFrame& setGetRecordMode(const GetRecordMode g);

         /**
          * Waits \a wait number of seconds, then reopens the file
          * and sets the next read as appropriate for the FileReadingMode.
          * @param wait number of seconds to sleep.
          */
      void waitForData(unsigned wait = 0);

         /// returns true if the file currently being read from has
         /// changed since its last read
      bool hasFileChanged();
      
         /// Returns true if all data for the given time has been read.
      bool endOfDataSet();

         /// end of data set iterator, useful for loops
      RTFileFrameIterator<FileStream, FileData> end();

         /// start of data set, set appropriately for the 
         /// given FileReadingMode.  if in AppendedData mode, begin will
         /// start at the next record that hasn't been read.  if in
         /// FromTheBeginning mode, begin starts at the start of the 
         /// current file.
      RTFileFrameIterator<FileStream, FileData> begin();

         /// reads the next record in the file
         /// @return true if a record was read, otherwise false.
      bool getRecord();

         /// when in Dumb mode, use this function to advance to the next day.
      void openNextDay();
      
         /// returns the last read data
      const FileData& data() const { return lastData;}

         /// this traverses the RTFileFrame from begin() to end(), adding
         /// each item to a list that gets returned. Using this will
         /// change the file position, so getRecord and any iterators
         /// will not be the same after this call.
      std::list<FileData> getList() const;

         /// this traverses the RTFileFrame from begin() to end(), adding
         /// each item to a FileFilter that gets returned. Using this will
         /// change the file position, so getRecord and any iterators
         /// will not be the same after this call.
      FileFilter<FileData> getFileFilter() const;

         /// returns the name of the current file begin used
      std::string getCurrentFile() const {return currentFileName;}

         /// returns the current time used for finding files
      gpstk::CommonTime getCurrentTime() const {return currentTime;}

         /// let the iterator see this class's insides
      friend class RTFileFrameIterator<FileStream, FileData>;

         /// reopens the file \a currentFileName
      bool openCurrentFile();

         /// close the file \a currentFileName
      void closeCurrentFile();

   protected:
         /// the internal file stream for the internal iterator
      FileStream* fileStream;

      FileData lastData;
         /// the last get position from the file
      std::streampos lastPosition;
         /// the last file to be read
      std::string currentFileName;
         /// the file spec for determining file names
      gpstk::FileSpec fs;
      
      gpstk::CommonTime startTime, ///< start time for file searching
         currentTime,            ///< time last used for finding a file
         endTime;                ///< end time for file searching
         /// the FileReadingMode for the reader
      FileReadingMode readMode;
         /// the GetRecordMode for the reader
      GetRecordMode getMode;
         /// keeps track of the last modify date for the last file read
      struct stat fileInfo;
         /// isOK is only false when the user needs to take action
         /// to continue reading - by calling waitForData(),
         /// openNextDay(), or openCurrentFile()
      bool isOK;
   };

   //@}

   template <class FileStream, class FileData>
   RTFileFrame<FileStream, FileData>::
   RTFileFrame<FileStream, FileData>(const gpstk::FileSpec& fnFormat,
                                     const gpstk::CommonTime& beginning,
                                     const gpstk::CommonTime& ending, 
                                     const RTFileFrameHelper::FileReadingMode frm,
                                     const RTFileFrameHelper::GetRecordMode grm)
      throw(gpstk::Exception)
         : fileStream(NULL), fs(fnFormat), startTime(beginning), 
           currentTime(beginning), endTime(ending), readMode(frm), getMode(grm)
   {
         // zero out seconds
      startTime = MJD(floor(MJD(startTime).mjd));
      endTime = MJD(floor(MJD(endTime).mjd));
      currentTime = MJD(floor(MJD(curretnTime).mjd));

         // set up the stream
      openCurrentFile();
   }

   template <class FileStream, class FileData>
   RTFileFrame<FileStream, FileData>::~RTFileFrame()
   {
      if (fileStream)
      {
         fileStream->close();
         delete fileStream;
      }
   }

   template <class FileStream, class FileData>
   RTFileFrame<FileStream, FileData>& 
   RTFileFrame<FileStream, FileData> :: 
   setFileReadingMode(const RTFileFrameHelper::FileReadingMode f)
   { 
     readMode = f; 
     return *this; 
   }

         /// Allows changing of the GetRecordMode
   template <class FileStream, class FileData>
   RTFileFrame<FileStream, FileData>& 
   RTFileFrame<FileStream, FileData> :: 
   setGetRecordMode(const RTFileFrameHelper::GetRecordMode g)
   { 
      getMode = g; 
      return *this;
   }

   template <class FileStream, class FileData>
   void
   RTFileFrame<FileStream, FileData>::waitForData(unsigned wait)
   {
      sleep(wait);
      if(readMode == AppendedData)
      {  
            // reopen the file and skip to where we left off - openCurrentFile
            // will overwrite lastPosition...
         std::streampos cachedPos = lastPosition;
         if(openCurrentFile())
            fileStream->seekg(cachedPos);
         lastPosition = cachedPos;
      }
      else
      {
            // start from the beginning
         openCurrentFile();
      }
   }

   template <class FileStream, class FileData>
   bool
   RTFileFrame<FileStream, FileData>::hasFileChanged()
   {
      struct stat tempStat;
      if (stat(currentFileName.c_str(), &tempStat))
         return false;
      if (tempStat.st_mtime != fileInfo.st_mtime)
         return true;
      else 
         return false;
   }

   template <class FileStream, class FileData>
   bool
   RTFileFrame<FileStream, FileData>::endOfDataSet()
   {
      if (currentTime > endTime)
         return true;
      else
         return false;
   }

   template <class FileStream, class FileData>
   RTFileFrameIterator<FileStream, FileData>
   RTFileFrame<FileStream, FileData>::end()
   {
      return RTFileFrameIterator<FileStream, FileData>();
   }

   template <class FileStream, class FileData>
   RTFileFrameIterator<FileStream, FileData> 
   RTFileFrame<FileStream, FileData>::begin()
   {
      if (readMode == AppendedData)
      {
         ;
      }
      else // if (readMode == FromTheBeginning)
      {
         openCurrentFile();
      }
         // this constructor does not read in the first record, so force it
         // to do so before returning
      RTFileFrameIterator<FileStream, FileData> toReturn(*this);
      return toReturn;
   }

   template <class FileStream, class FileData>
   bool
   RTFileFrame<FileStream, FileData>::getRecord()
   {
      if (!isOK)
         return false;

         // is the stream still good to read?
      if (*fileStream >> lastData)
      {
         lastPosition = fileStream->tellg();
         return true;
      }
         // the last read failed - try opening the next file until
         // today's file is reched or else return false
      else
      {
            // read the next file or not?
         if (getMode == Smart)
         {
            if (!endOfDataSet())
            {
                  // still before today?
               gpstk::CommonTime today;
               today.setYDoySod(today.year(), today.DOY(), 0);
               
               if (currentTime < today)
               {
                  openNextDay();
                  return getRecord();
               }
                  // we've read through today, so just wait until tomorrow's
                  // data comes in - the user needs to call waitForData()
                  // to continue
               else
                  isOK = false;
            }
            else
            {
               isOK = false;
            }
         } // if (getmode == smart)

            // in dumb mode, the user needs to call openNextDay or
            // openCurrentFile to continue.
         else
         {
            isOK = false;
         }
      } // if (isOK)
      return isOK;
   }

   template <class FileStream, class FileData>
   void
   RTFileFrame<FileStream, FileData>::openNextDay()
   {
         // open a new file for another day, if any.
      currentTime += gpstk::CommonTime::SEC_DAY;
      if (!endOfDataSet())
         openCurrentFile();
   }
   
   template <class FileStream, class FileData>
   std::list<FileData>
   RTFileFrame<FileStream, FileData>::getList() const
   {
      RTFileFrameIterator<FileStream, FileData> inputStream;
      std::list<FileData> toReturn;
      inputStream = begin();
      while (inputStream != end())
      {
         toReturn.push_back(data());
      }
      return toReturn;
   }

   template <class FileStream, class FileData>
   FileFilter<FileData>
   RTFileFrame<FileStream, FileData>::getFileFilter() const
   {
      RTFileFrameIterator<FileStream, FileData> inputStream;
      FileFilter<FileData> toReturn;
      inputStream = begin();
      while (inputStream != end())
      {
         toReturn.addData(data());
      }
      return toReturn;
   }

   template <class FileStream, class FileData>
   bool
   RTFileFrame<FileStream, FileData>::openCurrentFile()
   {
      isOK = false;
      currentFileName = fs.toString(currentTime);
      stat(currentFileName.c_str(), &fileInfo);
      lastPosition = 0;

      if(fileStream)
      {
         fileStream->close();
         delete fileStream;
         fileStream = NULL;
      }

      fileStream = new FileStream;
      fileStream->open(currentFileName.c_str(), std::ios::in);
      if (!fileStream->fail())
         isOK = true;
      return isOK;
   }

   template <class FileStream, class FileData>
   void
   RTFileFrame<FileStream, FileData>::closeCurrentFile()
   {
      if(fileStream)
      {
         fileStream->close();
         delete fileStream;
         fileStream = NULL;
      }
      isOK = false;
   }

} // namespace gpstk

#endif  // GPSTK_RTFILEFRAME_HPP
