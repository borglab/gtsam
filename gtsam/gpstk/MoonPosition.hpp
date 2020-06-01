#pragma ident "$Id$"

/**
 * @file MoonPosition.hpp
 * Returns the approximate position of the Moon at the given epoch in the
 * ECEF system.
 */

#ifndef MOONPOSITION_HPP
#define MOONPOSITION_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007
//
//============================================================================


#include <cmath>
#include <string>

#include "CommonTime.hpp"
#include "Triple.hpp"
#include "GNSSconstants.hpp"
#include "AstronomicalFunctions.hpp"
#include "Vector.hpp"
#include "Matrix.hpp"


namespace gpstk
{

      /** @addtogroup ephemcalc */
      //@{

      /** This class computes the approximate position of the Moon at
       *  the given epoch in the ECEF system. It is limited between
       *  March 1st, 1900 and February 28th, 2100.
       *
       * The class is based in the Meeus algorithms published in Meeus,
       * l'Astronomie, June 1984, p348. This is a C++ implementation version
       * of the FORTRAN version originally written by P.T. Wallace, Starlink
       * Project. The FORTRAN version of Starlink project was available under
       * the GPL license.
       *
       * Errors in position (RMS) are:
       *
       * \li Longitude: 3.7 arcsec.
       * \li Latitude: 2.3 arcsec.
       * \li Distance: 11 km.
       *
       * More information may be found in http://starlink.jach.hawaii.edu/
       */
   class MoonPosition
   {
   public:

         /// Default constructor
      MoonPosition() throw() {}

         /// Destructor
      virtual ~MoonPosition() {}


         /** Returns the position of Moon ECEF coordinates (meters) at the
          *  indicated time.
          *
          * @param[in]  t the time to look up
          *
          * @return the position of the Moon at time (as a Triple)
          *
          * @throw InvalidRequest If the request can not be completed for any
          *    reason, this is thrown. The text may have additional
          *    information as to why the request failed.
          *
          * @warning This method yields and approximate result, given
          *    that pole movement is not taken into account, neither
          *    precession nor nutation.
          */
      Triple getPosition(const CommonTime& t) const
         throw(InvalidRequest);


         /** Function to compute Moon position in CIS system (coordinates
          *  in meters)
          *
          * @param t Epoch
          */
      Triple getPositionCIS(const CommonTime& t) const
         throw(InvalidRequest);


         /** Determine the earliest time for which this object can
          *  successfully determine the position for the Moon.
          *
          * @return The initial time
          *
          * @throw InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getInitialTime() const throw(InvalidRequest)
      { return initialTime; }


         /** Determine the latest time for which this object can
          *  successfully determine the position for the Moon.
          *
          * @return The final time
          *
          * @throw InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getFinalTime() const throw(InvalidRequest)
      { return finalTime; }


   private:

         /// Time of the first valid time
      static const CommonTime initialTime;

         /// Time of the last valid time
      static const CommonTime finalTime;

         // Coefficients for fundamental arguments
         // Units are degrees for position and Julian centuries for time

         /// Moon's mean longitude
      static const double ELP0, ELP1, ELP2, ELP3;

         /// Sun's mean anomaly
      static const double EM0, EM1, EM2, EM3;

         /// Moon's mean anomaly
      static const double EMP0, EMP1, EMP2, EMP3;

         /// Moon's mean elongation
      static const double D0, D1, D2, D3;

         /// Mean distance of the Moon from its ascending node
      static const double F0, F1, F2, F3;

         /// Longitude of the Moon's ascending node
      static const double OM0, OM1, OM2, OM3;

         /// Coefficients for (dimensionless) E factor
      static const double E1, E2;

         /// Coefficients for periodic variations, etc
      static const double PAC, PA0, PA1;
      static const double PBC;
      static const double PCC;
      static const double PDC;
      static const double PEC, PE0, PE1, PE2;
      static const double PFC;
      static const double PGC;
      static const double PHC;
      static const double cPIC;
      static const double PJC, PJ0, PJ1;
      static const double CW1;
      static const double CW2;

         // Coefficients for Moon position
         //      Tx(N): coefficient of L, B or P term (deg)
         //      ITx(N,0-4): coefficients of M, M', D, F, E**n in argument
         //
      static const size_t NL, NB, NP;
      static Vector<double> TL, TB, TP;
      static Matrix<int> ITL, ITB, ITP;

   }; // end class MoonPosition


      //@}

} // namespace gpstk
#endif  // MOONPOSITION_HPP
