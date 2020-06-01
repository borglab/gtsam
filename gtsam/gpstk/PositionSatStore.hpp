/// @file PositionSatStore.hpp
/// Store a tabular list of ephemeris data (position, velocity, acceleration)
/// for several satellites, and compute values at any timetag from this table
/// using Lagrange interpolation. Inherits TabularSatStore.

#ifndef GPSTK_POSITION_SAT_STORE_INCLUDE
#define GPSTK_POSITION_SAT_STORE_INCLUDE

#include <map>
#include <iostream>

#include "TabularSatStore.hpp"
#include "Exception.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "Triple.hpp"
#include "SP3Data.hpp"

namespace gpstk
{

   /** @addtogroup ephemstore */
   //@{

   /// Data record for storing clock data. See note on units in class PositionStore.
   typedef struct PositionStoreDataRecord {
      Triple Pos, sigPos;  ///< position (ECEF Cartesian) and sigmas
      Triple Vel, sigVel;  ///< velocity and sigmas
      Triple Acc, sigAcc;  ///< acceleration and sigmas
   } PositionRecord;

   /// Output stream operator is used by dump() in TabularSatStore
   std::ostream& operator<<(std::ostream& os, const PositionRecord& cdr) throw();

   // This is a helper for SWIG processing - it needs a template instation of the
   // base type of PositionSatStore before it is used, so this statement must be between
   // the PositionStoreDataRecord and PositionSatStore declarations.
   #ifdef SWIG
   %template(TabularSatStore_PositionRecord) gpstk::TabularSatStore<gpstk::PositionRecord>;
   #endif

   /// Store a table of data vs time for each of several satellites.
   /// The data are stored as PositionRecords, one for each (satellite,time) pair.
   /// The getValue(sat, t) routine interpolates the table for sat at time t and
   /// returns the result as a DataRecord object.
   /// NB this class (dump()) requires that operator<<(DataRecord) be defined,
   ///   unless dump() is declared and defined in this class.
   /// NB. It is assumed that the units of the quanitities are 'coordinated'
   /// meaning that units(vel) == units(pos)/sec, units(acc) == units(pos)/sec/sec
   /// and units(sigX) == units(X). This assumption is critical only when
   /// interpolation is used to estimate X/sec from X data.
   /// No other assumptions are made about units.
   /// Note that SP3 data (in the file and in SP3Data) are NOT coordinated; users
   /// and derived classes must deal with units consistently.
   class PositionSatStore : public TabularSatStore<PositionRecord>
   {

   // member data
   protected:

      // NB havePosition and haveVelocity are in TabularSatStore
      /// flag indicating whether acceleration data is present
      bool haveAcceleration;

      /// Flag to reject bad positions, default true
      bool rejectBadPosFlag;

      /// Order of Lagrange interpolation used in interpolation of the data tables.
      /// Should be even; is forced to be even in setInterpolationOrder.
      /// Usually for 15min data, interpOrder = 10.
      unsigned int interpOrder;

      /// Store half the interpolation order, for convenience
      unsigned int Nhalf;

   // member functions
   public:

      /// Default constructor
      PositionSatStore() throw() : haveAcceleration(false), rejectBadPosFlag(true),
                                   Nhalf(5)
      {
         interpOrder = 2*Nhalf;
         havePosition = true;
         haveVelocity = false;
         haveClockBias = false;
         haveClockDrift = false;
      }

      /// Destructor
      ~PositionSatStore() {};

      /// Tabular does not have this...
      bool hasAccleration() const throw() { return haveAcceleration; }

      /// Return value for the given satellite at the given time (usually via
      /// interpolation of the data table). This interface from TabularSatStore.
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return object of type PositionRecord containing the data value(s).
      /// @throw InvalidRequest if data value cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      PositionRecord getValue(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Return the position for the given satellite at the given time
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return Triple containing the position ECEF XYZ meters
      /// @throw InvalidRequest if result cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      Triple getPosition(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Return the velocity for the given satellite at the given time
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return Triple containing the velocity ECEF XYZ meters/second
      /// @throw InvalidRequest if result cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      Triple getVelocity(const SatID& sat, const CommonTime& ttag)
         const throw(InvalidRequest);

      /// Return the acceleration for the given satellite at the given time
      /// @param[in] sat the SatID of the satellite of interest
      /// @param[in] ttag the time (CommonTime) of interest
      /// @return Triple containing the acceleration ECEF XYZ meters/second/second
      /// @throw InvalidRequest if result cannot be computed, for example because
      ///  a) the time t does not lie within the time limits of the data table
      ///  b) checkDataGap is true and there is a data gap
      ///  c) checkInterval is true and the interval is larger than maxInterval
      ///  d) neither velocity nor acceleration data are present
      Triple getAcceleration(const SatID& sat, const CommonTime& ttag)
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
         os << "Dump of PositionSatStore(" << detail << "):\n";
         os << " This store " << (haveAcceleration ? "contains":"does not contain")
            << " acceleration data." << std::endl;
         os << " Interpolation is Lagrange, of order " << interpOrder
            << " (" << Nhalf << " points on each side)" << std::endl;
         TabularSatStore<PositionRecord>::dump(os,detail);
         os << "End dump of PositionSatStore.\n";
      }

      /// Add a complete PositionRecord to the store; this is the preferred method
      /// of adding data to the tables.
      /// NB. If these addXXX() routines are used more than once for the same record
      /// (sat,ttag), be aware that since ttag is used as they key in a std::map,
      /// the value used must be EXACTLY the same in all calls; (numerical noise could
      /// cause the std::map to consider two "equal" ttags as different).
      void addPositionRecord(const SatID& sat, const CommonTime& ttag,
                             const PositionRecord& rec)
         throw(InvalidRequest);

      /// Add position data to the store; nothing else is changed
      void addPositionData(const SatID& sat, const CommonTime& ttag,
                           const Triple& Pos, const Triple& Sig=Triple())
         throw(InvalidRequest);

      /// Add velocity data to the store; nothing else is changed
      void addVelocityData(const SatID& sat, const CommonTime& ttag,
                           const Triple& Vel, const Triple& Sig=Triple())
         throw(InvalidRequest);

      /// Add acceleration data to the store; nothing else is changed
      void addAccelerationData(const SatID& sat, const CommonTime& ttag,
                               const Triple& Acc, const Triple& Sig=Triple())
         throw(InvalidRequest);

      /// Get current interpolation order.
      unsigned int getInterpolationOrder(void) throw()
         { return interpOrder; }

      /// Set the interpolation order; this routine forces the order to be even.
      void setInterpolationOrder(unsigned int order) throw()
         { Nhalf = (order+1)/2; interpOrder = 2*Nhalf; }

      /// Set the flag; if true then bad position values are rejected when
      /// adding data to the store.
      void rejectBadPositions(const bool flag)
         { rejectBadPosFlag=flag; }

   }; // end class PositionSatStore

      //@}

}  // End of namespace gpstk

#endif // GPSTK_POSITION_SAT_STORE_INCLUDE
