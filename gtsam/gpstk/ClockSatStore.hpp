/// @file ClockSatStore.hpp
/// Store a tabular list of clock data (bias, drift, accel) for several satellites,
/// and compute values at any timetag from this table. Inherits TabularSatStore.
/// This class is not virtual and could be used as is, however note there is no
/// FileStore, and no data file input (loadFile()), but there are addData() routines.

#ifndef GPSTK_CLOCK_SAT_STORE_INCLUDE
#define GPSTK_CLOCK_SAT_STORE_INCLUDE

#include <map>
#include <iostream>

#include "Exception.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "TabularSatStore.hpp"
#include "FileStore.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Data record for storing clock data. See note on units in class ClockSatStore.
   typedef struct ClockDataRecord {
      double bias, sig_bias;     ///< clock bias and sigma
      double drift, sig_drift;   ///< clock drift and sigma
      double accel, sig_accel;   ///< clock acceleration and sigma
   } ClockRecord;

   /// Output stream operator is used by dump() in TabularSatStore
   std::ostream& operator<<(std::ostream& os, const ClockRecord& rec) throw();

   // This is a helper for SWIG processing - it needs a template instation of the
   // base type of ClockSatStore before it is used, so this statement must be between
   // the ClockDataRecord and ClockSatStore declarations.
   #ifdef SWIG
   %template(TabularSatStore_ClockRecord) gpstk::TabularSatStore<gpstk::ClockRecord>;
   #endif

   /// Store a table of data vs time for each of several satellites.
   /// The data are stored as ClockRecords, one for each (satellite,time) pair.
   /// The getValue(sat, t) routine interpolates the table for sat at time t and
   /// returns the result as a DataRecord object.
   /// NB this class (dump()) requires that operator<<(DataRecord) be defined,
   ///   unless dump() is declared and defined in this class.
   /// NB. It is assumed that the units of the quanitities are 'coordinated'
   /// meaning that units(drift) == units(bias)/sec, units(acc) == units(drift)/s/s
   /// and units(sigX) == units(X). This assumption is critical only when
   /// interpolation is used to estimate X/sec from X data.
   /// No other assumptions are made about units.
   /// Note that SP3 data (in the file and in SP3Data) are NOT coordinated; users
   /// and derived classes must deal with units consistently.
   class ClockSatStore : public TabularSatStore<ClockRecord>
   {

   // member data
   protected:

      // NB haveClockBias and haveClockDrift are in TabularSatStore
      /// flag indicating whether acceleration data is present
      bool haveClockAccel;

      /// Type of interpolation: 1=linear, 2=Lagrange, 3=numerical derivative?
      int interpType;

      /// Order of Lagrange interpolation; should be even, is forced to be even
      /// in setInterpolationOrder (for linear interpolation it is 2).
      unsigned int interpOrder;

      /// Store half the interpolation order, for convenience
      unsigned int Nhalf;

      /// Flag to reject bad clock data; default true
      bool rejectBadClockFlag;

   // member functions
   public:

      /// Default constructor
      ClockSatStore() throw() : haveClockAccel(false),
                                interpType(2), Nhalf(5),
                                rejectBadClockFlag(true)
      {
         // NB. if interpType = 1, interpOrder = 2 (linear)
         interpOrder = 2*Nhalf;
         haveClockBias = true;
         haveClockDrift = havePosition = haveVelocity = false;
      }

      /// Destructor
      virtual ~ClockSatStore() {};

      ///
      bool hasClockAccel() const throw() { return haveClockAccel; }

      /// Return value for the given satellite at the given time (usually via
      /// interpolation of the data table). This interface from TabularSatStore.
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return object of type ClockRecord containing the data value(s).
      /// @throw InvalidRequest if data value cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      virtual ClockRecord getValue(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Return the clock bias for the given satellite at the given time
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return double the clock bias
      /// @throw InvalidRequest if bias cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      double getClockBias(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Return the clock drift for the given satellite at the given time
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return double the clock drift
      /// @throw InvalidRequest if drift cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      ///  d) there is no drift data in the store
      double getClockDrift(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Dump information about the object to an ostream.
      /// @param[in] os ostream to receive the output; defaults to std::cout
      /// @param[in] detail integer level of detail to provide; allowed values are
      ///    0: number of satellites, time step and time limits, flags,
      ///           gap and interval flags and values, and file information
      ///    1: number of data/sat
      ///    2: above plus all the data tables
      virtual void dump(std::ostream& os = std::cout, int detail = 0) const throw()
      {
         os << "Dump of ClockSatStore(" << detail << "):\n";
         os << " This store " << (haveClockAccel ? "contains":" does not contain")
            << " clock acceleration data." << std::endl;
         os << " Interpolation is ";
         if(interpType == 2) os << "Lagrange, of order " << interpOrder
            << " (" << Nhalf << " points on each side)" << std::endl;
         else                os << "Linear." << std::endl;
         TabularSatStore<ClockRecord>::dump(os,detail);
         os << "End dump of ClockSatStore.\n";
      }

      /// Add a complete ClockRecord to the store; this is the preferred method
      /// of adding data to the tables.
      /// NB. If these addXXX() routines are used more than once for the same record
      /// (sat,ttag), be aware that since ttag is used as they key in a std::map,
      /// the value used must be EXACTLY the same in all calls; (numerical noise could
      /// cause the std::map to consider two "equal" ttags as different).
      void addClockRecord(const SatID& sat, const CommonTime& ttag,
                          const ClockRecord& rec)
         throw(InvalidRequest);

      /// Add clock bias data (only) to the store
      void addClockBias(const SatID& sat, const CommonTime& ttag,
                        const double& bias, const double& sig=0.0)
         throw(InvalidRequest);

      /// Add clock drift data (only) to the store
      void addClockDrift(const SatID& sat, const CommonTime& ttag,
                        const double& drift, const double& sig=0.0)
         throw(InvalidRequest);

      /// Add clock acceleration data (only) to the store
      void addClockAcceleration(const SatID& sat, const CommonTime& ttag,
                        const double& accel, const double& sig=0.0)
         throw(InvalidRequest);

      /// Get current interpolation order.
      unsigned int getInterpolationOrder(void) throw()
         { return interpOrder; }

      /// Set the interpolation order; this routine forces the order to be even.
      void setInterpolationOrder(unsigned int order) throw()
      {
         if(interpType == 2) Nhalf = (order+1)/2;
         else                Nhalf = 1;
         interpOrder = 2*Nhalf;
      }

      /// Set the flag; if true then bad position values are rejected when
      /// adding data to the store.
      void rejectBadClocks(const bool flag)
         { rejectBadClockFlag = flag; }

      /// Set the type of interpolation to Lagrange (default)
      void setLagrangeInterp(void) throw()
         { interpType = 2; setInterpolationOrder(10); }

      /// Set the type of interpolation to linear. Note that
      void setLinearInterp(void) throw()
         { interpType = 1; setInterpolationOrder(2); }

   }; // end class ClockSatStore

      //@}

}  // End of namespace gpstk

#endif // GPSTK_CLOCK_SAT_STORE_INCLUDE
