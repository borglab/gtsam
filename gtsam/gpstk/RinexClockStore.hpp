#pragma ident "$Id$"

/// @file RinexClockStore.hpp
/// Store a tabular list of clock data (bias, drift, accel) for several satellites,
/// and compute values at any timetag from this table. Inherits ClockSatStore,
/// but assumes file input from RINEX Clock files.

#ifndef GPSTK_RINEX_CLOCK_STORE_INCLUDE
#define GPSTK_RINEX_CLOCK_STORE_INCLUDE

#include <iostream>
#include <string>

#include "Exception.hpp"
#include "ClockSatStore.hpp"
#include "FileStore.hpp"
#include "RinexClockStream.hpp"
#include "RinexClockHeader.hpp"
#include "RinexClockData.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Store a table of data vs time for each of several satellites using data from
   /// RINEX clock files. Most of the functionality is in ClockSatStore and
   /// TabularSatStore.
   class RinexClockStore : public ClockSatStore
   {

   // member data
   private:
      /// FileStore for the RINEX clock input files
      FileStore<RinexClockHeader> clkFiles;

   // member functions
   public:

      /// Default constructor
      RinexClockStore() throw()
      { }

      /// Destructor
      virtual ~RinexClockStore() {};

      /// Overload TabularSatStore::dump()
      /// Dump information about the object to an ostream.
      /// @param[in] os ostream to receive the output; defaults to std::cout
      /// @param[in] detail integer level of detail to provide; allowed values are
      ///    0: number of satellites, time step and time limits, flags,
      ///           gap and interval flags and values, and file information
      ///    1: number of data/sat
      ///    2: above plus all the data tables
      virtual void dump(std::ostream& os = std::cout, int detail = 0) const throw()
      {
         os << "Dump of RinexClockStore(" << detail << "):" << std::endl;
         if(detail >= 0) {
            const char *fmt="%4Y/%02m/%02d %2H:%02M:%02S";

            os << " Data stored for " << nsats()
               << " satellites\n";

            CommonTime initialTime(getInitialTime()), finalTime(getFinalTime());
            os << " Time span of data: ";
            if(initialTime == CommonTime::END_OF_TIME ||
               finalTime == CommonTime::BEGINNING_OF_TIME)
                  os << "(there are no time limits)" << std::endl;
            else
               os << initialTime.printf(fmt) << " TO "
                  << finalTime.printf(fmt) << std::endl;

            os << " This store contains:"
               << (haveClockBias ? "":" not") << " bias,"
               << (haveClockDrift ? "":" not") << " drift, and"
               << (haveClockAccel ? "":" not") << " acceleration data." << std::endl;
            os << " Checking for data gaps? " << (checkDataGap ? "yes":"no");
            if(checkDataGap) os << "; gap interval is "
               << std::fixed << std::setprecision(2) << gapInterval;
            os << std::endl;
            os << " Checking data interval? " << (checkInterval ? "yes":"no");
            if(checkInterval) os << "; max interval is "
               << std::fixed << std::setprecision(2) << maxInterval;
            os << std::endl;
            os << " Interpolation type is "
               << (interpType == 2 ? "Lagrange" : "Linear")
               << " using " << interpOrder << " consecutive data." << std::endl;
            os << " Rejecting bad clock data is turned "
               << (rejectBadClockFlag ? "ON":"OFF") << std::endl;

            // dump FileStore
            clkFiles.dump(os,detail);

            if(detail == 0) return;

            os << " List of satellites and number of records:\n";
            SatTable::const_iterator it;
            for(it=tables.begin(); it!=tables.end(); it++) {
               os << "  Sat " << it->first << " : "
                  << it->second.size() << " records.";

               if(detail == 1) { os << std::endl; continue; }

               os << "  Data:" << std::endl;
               DataTable::const_iterator jt;
               for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
                  os << " " << jt->first.printf(fmt)
                     << " " << it->first
                     << std::scientific << std::setprecision(12)
                     << " " << std::setw(19) << jt->second.bias
                     << " " << std::setw(19) << jt->second.sig_bias;
                  if(haveClockDrift) os
                     << " " << std::setw(19) << jt->second.drift
                     << " " << std::setw(19) << jt->second.sig_drift;
                  if(haveClockAccel) os
                     << " " << std::setw(19) << jt->second.accel
                     << " " << std::setw(19) << jt->second.sig_accel;
                  os << std::endl;
               }
            }
         }
         os << "End dump of RinexClockStore." << std::endl;
      }


      /// Load a RINEX clock file; may set the drift and accel flags.
      bool loadFile(const std::string& filename) throw(Exception)
      {
      try {
         // open the input stream
         RinexClockStream strm(filename.c_str());
         if(!strm.is_open()) {
            Exception e("File " + filename + " could not be opened");
            GPSTK_THROW(e);
         }
         strm.exceptions(std::ios::failbit);
         //cout << "Opened file " << filename << endl;

         // declare header and data
         RinexClockHeader head;
         RinexClockData data;

         // read the RINEX clock header
         try {
            strm >> head;
         }
         catch(Exception& e) {
            e.addText("Error reading header of file " + filename);
            GPSTK_RETHROW(e);
         }
         //cout << "Read header" << endl; head.dump();

         // save in FileStore
         clkFiles.addFile(filename, head);

         // read data
         try {
            while(strm >> data) {
               //data.dump(cout);

               if(data.datatype == std::string("AS")) {
                  // add this data
                  ClockRecord rec;
                  rec.bias = data.bias; rec.sig_bias = data.sig_bias,
                  rec.drift = data.drift; rec.sig_drift = data.sig_drift,
                  rec.accel = data.accel; rec.sig_accel = data.sig_accel;
                  addClockRecord(data.sat, data.time, rec);
               }
            }
         }
         catch(Exception& e) {
            e.addText("Error reading data of file " + filename);
            GPSTK_RETHROW(e);
         }

         // close
         strm.close();

      }
      catch(Exception& e) { GPSTK_RETHROW(e); }

      }  // end RinexClockStore::loadFile()

   }; // end class RinexClockStore

      //@}

}  // End of namespace gpstk

#endif // GPSTK_RINEX_CLOCK_STORE_INCLUDE

