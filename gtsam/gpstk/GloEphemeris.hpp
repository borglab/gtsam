/// @file GloEphemeris.hpp
/// Ephemeris data for GLONASS.

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2011
//
//============================================================================

#ifndef GPSTK_GLOEPHEMERIS_HPP
#define GPSTK_GLOEPHEMERIS_HPP

#include <iostream>
#include "Triple.hpp"
#include "Xvt.hpp"
#include "CommonTime.hpp"
#include "PZ90Ellipsoid.hpp"
#include "Vector.hpp"
#include "YDSTime.hpp"

namespace gpstk
{

      /** @addtogroup ephemcalc */
      //@{

      /**
       * Ephemeris information for a single GLONASS satellite.  This class
       * encapsulates the ephemeris navigation message and provides functions
       * to handle the ephemerides.
       */
   class GloEphemeris : public Xvt
   {
   public:

         /// Default constructor
      GloEphemeris()
         : valid(false), step(1.0)
      {};


         /// Destructor.
      virtual ~GloEphemeris() {};


         /// Query the presence of data in this object.
      bool isValid(short subframe) const
      { return valid; };


         /** Compute satellite position & velocity at the given time
          *  using this ephemeris data.
          *
          * @param epoch   Epoch to compute position and velocity.
          *
          * @throw InvalidRequest if required data has not been stored.
          */
      Xvt svXvt(const CommonTime& epoch) const
         throw( gpstk::InvalidRequest );


         /// Get the epoch time for this ephemeris
      CommonTime getEphemerisEpoch() const
         throw( gpstk::InvalidRequest );


         /// Get the epoch time for this ephemeris
      CommonTime getEpochTime() const
         throw( gpstk::InvalidRequest )
      { return getEphemerisEpoch(); };


         /** This functions returns the GNSS type (satellite system code) */
      std::string getSatSys() const
         throw()
      { return satSys; };


         /// This function returns the PRN ID of the SV.
      short getPRNID() const
         throw( gpstk::InvalidRequest );


         /** Compute the satellite clock bias (sec) at the given time
          *
          * @param epoch   Epoch to compute satellite clock bias.
          *
          * @throw InvalidRequest if required data has not been stored.
          */
      double svClockBias(const CommonTime& epoch) const
         throw( gpstk::InvalidRequest );


         /** Compute the satellite clock drift (sec/sec) at the given time
          *
          * @param epoch   Epoch to compute satellite clock drift.
          *
          * @throw InvalidRequest if required data has not been stored.
          */
      double svClockDrift(const CommonTime& t) const
         throw( gpstk::InvalidRequest );


         /// Get integration step for Runge-Kutta algorithm.
      double getIntegrationStep() const
      { return step; };


         /** Set integration step for Runge-Kutta algorithm.
          *
          * @param rkStep  Runge-Kutta integration step in seconds.
          */
      GloEphemeris& setIntegrationStep( double rkStep )
      { step = rkStep; return (*this); };


         /// Get the acceleration vector.
      Triple getAcc() const
         throw()
      { return a; }


         /// Get the TauN parameter.
      double getTauN() const
         throw()
      { return clkbias; }


         /// Get the GammaN parameter.
      double getGammaN() const
         throw()
      { return clkdrift; }


         /// Get the MFTime parameter.
      long getMFtime() const
         throw()
      { return MFtime; }


         /// Get the health value parameter.
      short getHealth() const
         throw()
      { return health; }


         /// Get the frequency number.
      short getfreqNum() const
         throw()
      { return freqNum; }


         /// Get the age of the information.
      double getAgeOfInfo() const
         throw()
      { return ageOfInfo; }


         /// Output the contents of this ephemeris to the given stream.
      void dump(std::ostream& s = std::cout) const
         throw();
         
      void prettyDump(std::ostream& s) const;
      void terseDump(std::ostream& s) const;
      void terseHeader(std::ostream& s) const;

         /// Set the parameters for this ephemeris object.
      GloEphemeris& setRecord( std::string svSys,
                               short prn,
                               const CommonTime& epoch,
                               Triple pos,
                               Triple vel,
                               Triple acc,
                               double clkbias,
                               double clkdrift,
                               long mftime,
                               short h,
                               short freqnum,
                               double ageofinfo,
                               double rkStep = 1.0 );


   protected:


      std::string satSys;  ///< GNSS (satellite system)
      short PRNID;         ///< SV PRN ID
      CommonTime ephTime;  ///< Epoch for this ephemeris
      Triple a;            ///< SV acceleration (x,y,z), Earth-fixed [meters]
      long MFtime;        ///< Message frame time [sec of UTC week]
      short health;        ///< SV health
      short freqNum;       ///< Frequency (channel) number (-7..+12)
      double ageOfInfo;    ///< Age of oper. information [days]


   private:


         /// Flag indicating that this object has valid data.
      bool valid;

      
         /// Integration step for Runge-Kutta algorithm (1 second by default)
      double step;


         /// Compute true sidereal time (in hours) at Greenwich at 0 hours UT.
      double getSidTime( const CommonTime& time ) const;


         /// Function implementing the derivative of GLONASS orbital model.
      Vector<double> derivative( const Vector<double>& inState,
                                 const Vector<double>& accel ) const;




         /// Output the contents of this ephemeris to the given stream.
      friend std::ostream& operator<<( std::ostream& s,
                                       const GloEphemeris& glo );

      //@}

  };  // End of class 'GloEphemeris'

}  // End of namespace gpstk

#endif   // GPSTK_GLOEPHEMERIS_HPP
