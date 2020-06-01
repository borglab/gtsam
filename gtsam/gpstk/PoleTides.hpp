#pragma ident "$Id$"

/**
 * @file PoleTides.hpp
 * Computes the effect of pole tides at a given position and epoch.
 */

#ifndef POLETIDES_HPP
#define POLETIDES_HPP

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


#include <cmath>
#include <string>

#include "Triple.hpp"
#include "Position.hpp"
#include "CommonTime.hpp"
#include "GNSSconstants.hpp"
#include "geometry.hpp"



namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{

      /** This class computes the effect of pole tides, or more properly
       *  called "rotational deformations due to polar motion", at a given
       *  position and epoch.
       *
       *  The model used is the one proposed by the "International Earth
       *  Rotation and Reference Systems Service" (IERS) in its upcomming
       *  "IERS Conventions" document (Chapter 7), available at:
       *
       *  http://tai.bipm.org/iers/convupdt/convupdt.html
       *
       *  The pole movement parameters x, y for a given epoch may be
       *  found at:
       *
       *  ftp://hpiers.obspm.fr/iers/eop/eop.others
       *
       *  Maximum displacements because of this effect are:
       *
       *  \li Vertical:    2.5 cm
       *  \li Horizontal:  0.7 cm
       *
       *  For additional information you may consult: Wahr, J.M., 1985,
       *  "Deformation Induced by Polar Motion", Journal of Geophysical
       *  Research, Vol. 90, No B11, p. 9363-9368.
       *
       *  \warning Please take into account that pole tide equations in
       *  IERS document use co-latitude instead of latitude.
       */
   class PoleTides
   {
   public:

         /// Default constructor. Sets zero pole displacement
      PoleTides() : xdisp(0.0), ydisp(0.0) {};


         /** Common constructor
          *
          * @param x     Pole displacement x, in arcseconds
          * @param y     Pole displacement y, in arcseconds
          */
      PoleTides( const double& x,
                 const double& y )
         : xdisp(x), ydisp(y) {};


         /** Returns the effect of pole tides (meters) at the given
          *  position and epoch, in the Up-East-North (UEN) reference frame.
          *
          * @param[in]  t Epoch to look up
          * @param[in]  p Position of interest
          *
          * @return a Triple with the pole tide effect, in meters and in
          *    the UEN reference frame.
          *
          * @throw InvalidRequest If the request can not be completed for any
          *    reason, this is thrown. The text may have additional
          *    information about the reason the request failed.
          *
          * @warning In order to use this method, you must have previously
          *    set the current pole displacement parameters.
          *
          */
      Triple getPoleTide( const CommonTime& t,
                          const Position& p )
         throw(InvalidRequest);


         /** Returns the effect of pole tides (meters) on the given
          *  position, in the Up-East-North (UEN) reference frame.
          *
          * @param[in] p Position of interest
          * @param[in] x Pole displacement x, in arcseconds
          * @param[in] y Pole displacement y, in arcseconds
          *
          * @return a Triple with the pole tide effect, in meters and in
          *    the UEN reference frame.
          *
          * @throw InvalidRequest If the request can not be completed for
          *    any reason, this is thrown. The text may have additional
          *    information about the reason the request failed.
          */
      Triple getPoleTide( const CommonTime& t,
                          const Position& p,
                          const double& x,
                          const double& y )
         throw(InvalidRequest)
      { setXY(x,y); return (getPoleTide(t, p)); };


         /** Method to set the pole displacement parameters
          *
          * @param x     Pole displacement x, in arcseconds
          * @param y     Pole displacement y, in arcseconds
          *
          * @return This same object
          */
      PoleTides& setXY( const double& x,
                        const double& y );


         /// Method to get the x pole displacement parameter, in arcseconds
      double getX(void) const
      { return xdisp; };


         /// Method to get the y pole displacement parameter, in arcseconds
      double getY(void) const
      { return ydisp; };


         /// Destructor
      virtual ~PoleTides() {};


   private:


         /// Pole displacement x, in arcseconds
      double xdisp;


         /// Pole displacement y, in arcseconds
      double ydisp;


   }; // End of class 'PoleTides'

      //@}

}  // End of namespace gpstk
#endif   // POLETIDES_HPP
