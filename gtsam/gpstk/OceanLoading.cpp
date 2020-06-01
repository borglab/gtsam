#pragma ident "$Id$"

/**
 * @file OceanLoading.cpp
 * This class computes the effect of ocean tides at a given position
 * and epoch.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008
//
//============================================================================


#include "OceanLoading.hpp"
#include "YDSTime.hpp"

using namespace std;

namespace gpstk
{


      /* Returns the effect of ocean tides loading (meters) at the given
       * station and epoch, in the Up-East-North (UEN) reference frame.
       *
       * @param name  Station name (case is NOT relevant).
       * @param time  Epoch to look up
       *
       * @return a Triple with the ocean tidas loading effect, in meters and
       * in the UEN reference frame.
       *
       * @throw InvalidRequest If the request can not be completed for any
       * reason, this is thrown. The text may have additional information
       * about the reason the request failed.
       */
   Triple OceanLoading::getOceanLoading( const string& name,
                                         const CommonTime& t )
      throw(InvalidRequest)
   {

      const int NUM_COMPONENTS = 3;
      const int NUM_HARMONICS = 11;

      Matrix<double> harmonics(6,11,0.0);

         // Get harmonics data from file
      harmonics = blqData.getTideHarmonics(name);

      Vector<double> arguments(11,0.0);

         // Compute arguments
      arguments = getArg(t);

      Triple oLoading;

      for(int i=0; i<NUM_COMPONENTS;  i++)
      {

         double temp(0.0);
         for(int k=0; k<NUM_HARMONICS; k++)
         {

            temp += harmonics(i,k) * 
                    std::cos( arguments(k) - harmonics( (i+3),k)*DEG_TO_RAD );

         }

            // This Triple is in Up, West, South reference frame
         oLoading[i] = temp;

      }  // End of 'for(int i=0; i<NUM_COMPONENTS;  i++)'

         // Let's change Triple to Up, East, North [UEN] reference frame
      oLoading[1] = -oLoading[1];
      oLoading[2] = -oLoading[2];

      return oLoading;

   }  // End of method 'OceanLoading::getOceanLoading()'



      /* Sets the name of BLQ file containing ocean tides harmonics data.
       *
       * @param name      Name of BLQ tides harmonics data file.
       */
   OceanLoading& OceanLoading::setFilename(const string& name)
   {

      fileData = name;

      blqData.open(fileData);

      return (*this);

   }  // End of meters 'OceanLoading::setFilename()'



      /* Compute the value of the corresponding astronomical arguments,
       * in radians. This routine is based on IERS routine ARG.f.
       *
       * @param time      Epoch of interest
       *
       * @return A Vector<double> of 11 elements with the corresponding
       * astronomical arguments to be used in ocean loading model.
       */
   Vector<double> OceanLoading::getArg(const CommonTime& time)
   {

      const int NUM_HARMONICS = 11;

         // Let's store some important values
      Vector<double> sig(NUM_HARMONICS,0.0);
      sig(0) = 1.40519e-4;
      sig(1) = 1.45444e-4;
      sig(2) = 1.37880e-4;
      sig(3) = 1.45842e-4;
      sig(4) = 0.72921e-4;
      sig(5) = 0.67598e-4;
      sig(6) = 0.72523e-4;
      sig(7) = 0.64959e-4;
      sig(8) = 0.053234e-4;
      sig(9) = 0.026392e-4;
      sig(10)= 0.003982e-4;

      Matrix<double> angfac(4,NUM_HARMONICS,0.0);
      angfac(0,0) =  2.0; angfac(1,0) = -2.0;
         angfac(2,0) =  0.0; angfac(3,0) =  0.0;
      angfac(0,1) =  0.0; angfac(1,1) =  0.0;
         angfac(2,1) =  0.0; angfac(3,1) =  0.0;
      angfac(0,2) =  2.0; angfac(1,2) = -3.0;
         angfac(2,2) =  1.0; angfac(3,2) =  0.0;
      angfac(0,3) =  2.0; angfac(1,3) =  0.0;
         angfac(2,3) =  0.0; angfac(3,3) =  0.0;
      angfac(0,4) =  1.0; angfac(1,4) =  0.0;
         angfac(2,4) =  0.0; angfac(3,4) =  0.25;
      angfac(0,5) =  1.0; angfac(1,5) = -2.0;
         angfac(2,5) =  0.0; angfac(3,5) = -0.25;
      angfac(0,6) = -1.0; angfac(1,6) =  0.0;
         angfac(2,6) =  0.0; angfac(3,6) = -0.25;
      angfac(0,7) =  1.0; angfac(1,7) = -3.0;
         angfac(2,7) =  1.0; angfac(3,7) = -0.25;
      angfac(0,8) =  0.0; angfac(1,8) =  2.0;
         angfac(2,8) =  0.0; angfac(3,8) =  0.0;
      angfac(0,9) =  0.0; angfac(1,9) =  1.0;
         angfac(2,9) = -1.0; angfac(3,9) =  0.0;
      angfac(0,10)=  2.0; angfac(1,10)=  0.0;
         angfac(2,10)=  0.0; angfac(3,10)=  0.0;


      Vector<double> arguments(NUM_HARMONICS,0.0);

         // Get day of year
      short year(static_cast<YDSTime>(time).year);

         // Fractional part of day, in seconds
      double fday(static_cast<YDSTime>(time).sod);

         // Compute time
      double d(static_cast<YDSTime>(time).doy+365.0*(year-1975.0)+floor((year-1973.0)/4.0));
      double t((27392.500528+1.000000035*d)/36525.0);

         // Mean longitude of Sun at beginning of day
      double H0((279.69668+(36000.768930485+3.03e-4*t)*t)*DEG_TO_RAD);

         // Mean longitude of Moon at beginning of day
      double S0( (((1.9e-6*t - 0.001133)*t +
                    481267.88314137)*t + 270.434358)*DEG_TO_RAD );

         // Mean longitude of lunar perigee at beginning of day
      double P0( (((-1.2e-5*t - 0.010325)*t +
                    4069.0340329577)*t + 334.329653)*DEG_TO_RAD );

      for(int k=0; k<NUM_HARMONICS; k++)
      {

         double temp( sig(k)*fday + angfac(0,k)*H0 + 
                      angfac(1,k)*S0 + angfac(2,k)*P0 + angfac(3,k)*TWO_PI );

         arguments(k) = fmod(temp,TWO_PI);

         if (arguments(k) < 0.0)
         {
            arguments(k) = arguments(k) + TWO_PI;
         }

      }  // End of 'for(int k=0; k<NUM_HARMONICS; k++)'

      return arguments;

   }  // End of method 'OceanLoading::getArg()'



}  // End of namespace gpstk
