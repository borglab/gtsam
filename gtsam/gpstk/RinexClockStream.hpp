#pragma ident "$Id$"

/**
 * @file RinexClockStream.hpp
 * gpstk::RinexClockStream - RINEX Clock format file stream
 */

#ifndef RINEX_CLOCK_STREAM_INCLUDE
#define RINEX_CLOCK_STREAM_INCLUDE

#include <iostream>
#include <fstream>

#include "FFTextStream.hpp"

namespace gpstk
{
   /// @addtogroup RinexClock
   //@{

       /// This class performs file I/O on an RINEX Clock format file for the
       /// RinexClockHeader and RinexClockData classes.
   class RinexClockStream : public FFTextStream
   {
   public:
         /// Default constructor
      RinexClockStream() 
            : headerRead(false)
         {}
      
         /// Common constructor: open (default: read mode)
         /// @param filename the name of the ASCII RinexClock format file to be opened
         /// @param mode the ios::openmode to be used
      RinexClockStream(const char* filename, std::ios::openmode mode=std::ios::in)
            : FFTextStream(filename, mode), headerRead(false)
            { }

         /// destructor; override to force 'close'
      virtual ~RinexClockStream() { }

         /// override open() to reset the header
         /// @param filename the name of the ASCII RINEX Clock format file
         /// @param mode the ios::openmode to be used
      virtual void open(const char* filename, std::ios::openmode mode)
      {
         FFTextStream::open(filename, mode);
         headerRead = false;
      }

         ///@name data members
         //@{
      bool headerRead;             ///< true if the header has been read
         //@}

   }; // class RinexClockStream
   
   //@}
   
} // namespace gpstk

#endif // RINEX_CLOCK_STREAM_INCLUDE
