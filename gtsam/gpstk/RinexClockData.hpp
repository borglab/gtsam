#pragma ident "$Id$"

/**
 * @file RinexClockData.hpp
 * Encapsulate RinexClock file data, including I/O
 */

#ifndef GPSTK_RINEX_CLOCK_DATA_INCLUDE
#define GPSTK_RINEX_CLOCK_DATA_INCLUDE

#include "RinexSatID.hpp"
#include "RinexClockBase.hpp"
#include "CommonTime.hpp"
#include <iomanip>

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

      /**
       * This class encapsulates data for satellite clocks as found in RINEX Clock
       * format files, and is used in conjuction with class RinexClockStream,
       * which handles the I/O, and RinexClockHeader, which holds information
       * from the RinexClock file header.
       *
       * @code
       * RinexClockStream ss("igs14080.clk_30s");
       * RinexClockHeader sh;
       * RinexClockData sd;
       *
       * ss >> sh;
       *
       * while (ss >> sd) {
       *    // Interesting stuff...
       * }    
       *
       * RinexClockStream ssout("myfile.clk_30s", ios::out);
       * ssout << sh;
       * for(...) {
       *    // perhaps modify sd
       *    ssout << sd
       * }
       * @endcode
       *
       * @sa gpstk::RinexClockHeader and gpstk::RinexClockStream for more information.
       */
   class RinexClockData : public RinexClockBase
   {
   public:
         /// Constructor.
      RinexClockData() { clear(); }
     
         /// Destructor
      virtual ~RinexClockData() {}
     
         // The next four lines is our common interface
         /// RinexClockData is "data" so this function always returns true.
      virtual bool isData() const {return true;}

         /// Debug output function.
      virtual void dump(std::ostream& s=std::cout) const throw();

         ///@name data members
         //@{
      std::string datatype;   ///< Data type : AR, AS, etc
      RinexSatID sat;         ///< Satellite ID        (if AS)
      std::string site;       ///< Site label (4-char) (if AR)
      CommonTime time;        ///< Time of epoch for this record
      double bias;            ///< Clock bias in seconds
      double sig_bias;        ///< Clock bias sigma in seconds
      double drift;           ///< Clock drift in sec/sec
      double sig_drift;       ///< Clock drift sigma in sec/sec
      double accel;           ///< Clock acceleration in 1/sec
      double sig_accel;       ///< Clock acceleration sigma in 1/sec
         //@}
      
   protected:

      void clear(void) throw()
      {
         datatype = std::string();
         sat = RinexSatID(-1,RinexSatID::systemGPS);
         time = CommonTime::BEGINNING_OF_TIME;
         bias = sig_bias = drift = sig_drift = accel = sig_accel = 0.0;
      }

         /// Writes the formatted record to the FFStream \a s.
         /// @warning This function is currently unimplemented
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);

         /**
          * This function reads a record from the given FFStream.
          * If an error is encountered in retrieving the record, the 
          * stream is reset to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);
   };

   //@}

}  // namespace

#endif // GPSTK_RINEX_CLOCK_DATA_INCLUDE
