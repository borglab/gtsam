#pragma ident "$Id$"

/**
 * @file MOPSWeight.cpp
 * Class to assign weights to satellites based on the Appendix J of MOPS
 * document RTCA DO-229D.
 */


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
//  Dagoberto Salazar - gAGE. 2006, 2008, 2011
//
//============================================================================



#include "MOPSWeight.hpp"


using namespace std;

namespace gpstk
{


      /* Computes a vector with the weights for the given satellites.
       *
       * @param time               Epoch weights will be computed for.
       * @param Satellites         Vector of satellites.
       * @param bcEph              Satellite broadcast ephemeris.
       * @param ionoCorrections    Ionospheric corrections computed using
       *                           Klobuchar model.
       * @param elevationVector    Vector of elevations, in degrees.
       * @param azimuthVector      Vector of azimuths, in degrees.
       * @param rxPosition         Position of the receiver.
       * @param rxClass            Integer indicating receiver class
       *                           according MOPS-C. It is 2 by default
       *                           (conservative setting).
       *
       * @return Number of satellites with valid weights.
       *
       * \note
       * Method isValid() will return 'false' if some satellite does not have
       * a valid weight. Also, its PRN will be set to a negative value.
       *
       */
   int MOPSWeight::getWeights( CommonTime& time,
                               Vector<SatID>& Satellites,
                               GPSEphemerisStore& bcEph,
                               Vector<double>& ionoCorrections,
                               Vector<double>& elevationVector,
                               Vector<double>& azimuthVector,
                               Position rxPosition,
                               int rxClass )
      throw(InvalidWeights)
   {

      int N( Satellites.size() );
      int Ne( elevationVector.size() );
      int Na( azimuthVector.size() );

         // We need at least one satellite
      if (N == 0)
      {

         InvalidWeights eWeight("At least one satellite is needed to \
compute weights.");

         GPSTK_THROW(eWeight);

      }


      if ( (N != Ne) ||
           (N != Na) )
      {
         InvalidWeights eWeight("Size of input vectors do not match.");

         GPSTK_THROW(eWeight);

      }


      SimpleIURAWeight sIura;

         // Compute satellites' IURA weights
      int goodSV( sIura.getWeights(time, Satellites, bcEph) );

         // Let's compute sigma^2 and weight values
      Compute( goodSV,
               sIura,
               Satellites,
               ionoCorrections,
               elevationVector,
               azimuthVector,
               rxPosition,
               rxClass );

       return goodSV;

   }  // End of method 'MOPSWeight::getWeights()'



      /* Computes a vector with the weights for the given satellites.
       *
       * @param time               Epoch weights will be computed for.
       * @param Satellites         Vector of satellites.
       * @param preciseEph         Satellite precise ephemeris.
       * @param ionoCorrections    Ionospheric corrections computed using
       *                           Klobuchar model.
       * @param elevationVector    Vector of elevations, in degrees.
       * @param azimuthVector      Vector of azimuths, in degrees.
       * @param rxPosition         Position of the receiver.
       * @param rxClass            Integer indicating receiver class.
       *                           according MOPS-C. It is 2 by default
       *                           (conservative setting).
       *
       * @return Number of satellites with valid weights.
       *
       * \note
       * Method isValid() will return 'false' if some satellite does not have
       * a valid weight. Also, its PRN will be set to a negative value.
       *
       */
   int MOPSWeight::getWeights( CommonTime& time,
                               Vector<SatID>& Satellites,
                               TabularSatStore<Xvt>& preciseEph,
                               Vector<double>& ionoCorrections,
                               Vector<double>& elevationVector,
                               Vector<double>& azimuthVector,
                               Position rxPosition,
                               int rxClass )
      throw(InvalidWeights)
   {

      int N( Satellites.size() );
      int Ne( elevationVector.size() );
      int Na( azimuthVector.size() );

         // We need at least one satellite
      if (N == 0)
      {

         InvalidWeights eWeight("At least one satellite is needed to \
compute weights.");

         GPSTK_THROW(eWeight);

      }


      if ( (N != Ne) ||
           (N != Na) )
      {

         InvalidWeights eWeight("Size of input vectors do not match.");

         GPSTK_THROW(eWeight);

      }


      SimpleIURAWeight sIura;

         // Compute satellites' IURA weights
      int goodSV( sIura.getWeights(time, Satellites, preciseEph) );

         // Let's compute sigma^2 and weight values
      Compute( goodSV,
               sIura,
               Satellites,
               ionoCorrections,
               elevationVector,
               azimuthVector,
               rxPosition,
               rxClass );

      return goodSV;

   }  // End of method 'MOPSWeight::getWeights()'



      // Compute satellites' weights
   void MOPSWeight::Compute( int goodSV,
                             SimpleIURAWeight& sIura,
                             Vector<SatID>& Satellites,
                             Vector<double>& ionoCorrections,
                             Vector<double>& elevationVector,
                             Vector<double>& azimuthVector,
                             Position rxPosition,
                             int rxClass )
      throw(InvalidWeights)
   {

      int N( Satellites.size() );

      double sigma2rx;    // Receiver noise sigma^2 in meters^2

      if (rxClass==1)
      {
         sigma2rx = 0.25;
      }
      else
      {
         sigma2rx = 0.36;
      }

      double sigma2ura, sigma2multipath, sigma2trop, sigma2uire;

         // Let's resize weightsVector
      weightsVector.resize(goodSV);

         // We need a MOPSTropModel object. Parameters must be valid but
         // they aren't important
      MOPSTropModel mopsTrop(0.0, 0.0, 1);

      if (N==goodSV)    // In this case, we don't have to worry much
      {                 // and things go faster

         for (int i=0; i<goodSV; i++)
         {

            sigma2ura = (1.0 / sIura.weightsVector(i));
            sigma2multipath = 0.13 + ( 0.53 *
                                       std::exp(-elevationVector(i)/10.0) );
               // The former expression in DO-229D document is for sigma,
               // not for sigma^2. Thanks to Everett Wang for the fix.
            sigma2multipath *= sigma2multipath;
            sigma2trop = mopsTrop.MOPSsigma2(elevationVector(i));
            sigma2uire = sigma2iono( ionoCorrections(i),
                                     elevationVector(i),
                                     azimuthVector(i),
                                     rxPosition );
            weightsVector(i) = 1.0 / ( sigma2rx + sigma2ura + sigma2multipath
                                     + sigma2trop + sigma2uire );

         }  // End of 'for (int i=0; i<goodSV; i++)...'

      }
      else    // More care must be taken in this case
      {

         int offset(0);

         for (int i=0; i<goodSV; i++)
         {

               // Lets make sure we are using the same satellites
            while ( (sIura.availableSV(i).id != Satellites(i+offset).id) &&
                    ((i+offset)<N) )
            {
               offset++;
            }

            if ( (i+offset) >= N )
            {
               break;
            }

            sigma2ura = (1.0 / sIura.weightsVector(i));
            sigma2multipath = 0.13 + ( 0.53 *
                                       std::exp(-elevationVector(i)/10.0) );
               // The former expression in DO-229D document is for sigma,
               // not for sigma^2. Thanks to Everett Wang for the fix.
            sigma2multipath *= sigma2multipath;
            sigma2trop = mopsTrop.MOPSsigma2(elevationVector(i+offset));
            sigma2uire = sigma2iono( ionoCorrections(i+offset),
                                     elevationVector(i+offset),
                                     azimuthVector(i+offset),
                                     rxPosition );
            weightsVector(i) = 1.0 / ( sigma2rx + sigma2ura + sigma2multipath
                                     + sigma2trop + sigma2uire );

         }  // End of 'for (int i=0; i<goodSV; i++)...'

      }  // End of 'if (N==goodSV) ... else ...'


      valid = sIura.isValid();      // Valididy depends if sIura rejected
                                    // satellites or not
      availableSV = sIura.availableSV;
      rejectedSV = sIura.rejectedSV;

      return;

    } // End of method 'MOPSWeight::Compute()'



      // Compute ionospheric sigma^2 according to Appendix J.2.3 and
      // Appendix A.4.4.10.4 in MOPS-C
   double MOPSWeight::sigma2iono( double& ionoCorrection,
                                  double& elevation,
                                  double& azimuth,
                                  Position rxPosition )
      throw(InvalidWeights)
   {

         // First, let's found magnetic latitude according to ICD-GPS-200,
         // section 20.3.3.5.2.6
      double azRad( azimuth * DEG_TO_RAD );
      double elevRad( elevation * DEG_TO_RAD );
      double cosElev( std::cos(elevRad) );
      double svE( elevation / 180.0 );     // Semi-circles

      double phi_u( rxPosition.getGeodeticLatitude() / 180.0 ); // Semi-circles
      double lambda_u( rxPosition.getLongitude() / 180.0 );     // Semi-circles

      double psi( (0.0137 / (svE + 0.11)) - 0.022 );            // Semi-circles

      double phi_i( phi_u + psi * std::cos(azRad) );            // Semi-circles

      if (phi_i > 0.416)
      {
         phi_i = 0.416;
      }

      if (phi_i < -0.416)
      {
         phi_i = -0.416;
      }

         // In semi-circles
      double lambda_i( lambda_u + (psi*std::sin(azRad) / std::cos(phi_i*PI)) );
      double phi_m( phi_i + 0.064 * std::cos((lambda_i - 1.617)*PI) );

         // Convert magnetic latitude to degrees
      phi_m = std::abs(phi_m * 180.0);

         // Estimate vertical ionospheric delay according to MOPS-C
      double tau_vert;

      if ( (phi_m >= 0.0) &&
           (phi_m <= 20.0) )
      {
         tau_vert = 9.0;
      }
      else
      {

         if ( (phi_m > 20.0) &&
              (phi_m <= 55.0) )
         {
            tau_vert = 4.5;
         }
         else
         {
            tau_vert = 6.0;
         }

      }  // End of 'if ( (phi_m >= 0.0) && (phi_m <= 20.0) ) ... else ...'


         // Compute obliquity factor
      double fpp( 1.0 / (std::sqrt(1.0 - 0.898665418 * cosElev * cosElev)) );

      double sigma2uire( (ionoCorrection*ionoCorrection) / 25.0 );

      double fact( (fpp*tau_vert) * (fpp*tau_vert) );

      if (fact > sigma2uire)
      {
         sigma2uire = fact;
      }

      return sigma2uire;

   }  // End of method 'MOPSWeight::sigma2iono()'


}  // End of namespace gpstk
