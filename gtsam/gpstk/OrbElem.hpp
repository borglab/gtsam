#pragma ident "$Id$"

/**
 * @file OrbElem.hpp
 *  This class encapsulates the "least common denominator"
 *  orbit parameters defined in the GPS signal interface specifications.
 *  That is to say, the clock correction coefficients, the pseudo-
 *  Keplerian orbit parameters, the harmonic perturbations, and
 *  the associated times.
 *
 *  Generally, the user will want to instantiate a descendent of this
 *  class as opposed to instantiating this class directly.  The
 *  descendent classes provide the functionality to load the
 *  coefficients from various navigation message formats
 *  and types.
 */

#ifndef GPSTK_ORBELEM_HPP
#define GPSTK_ORBELEM_HPP

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
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S.
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software.
//
//Pursuant to DoD Directive 523024
//
// DISTRIBUTION STATEMENT A: This software has been approved for public
//                           release, distribution is unlimited.
//
//=============================================================================


#include "ObsID.hpp"
#include "Exception.hpp"
#include "CommonTime.hpp"
#include "Xvt.hpp"
#include "GNSSconstants.hpp"
#include "SatID.hpp"
#include "ObsID.hpp"
#include "MathBase.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"

namespace gpstk
{
   class OrbElem
   {
   public:
         /// Constructors
	      /// Default constuctor
      OrbElem( );


         /// Destructor
      virtual ~OrbElem() {}

         /// Clone method.
	 /// Return pointer to new copy of this type.
	 /// Implication of the "= 0" at this end is that this is a
	 /// "pure virtual" method and that makes OrbElem an abstract
	 /// class.  That is to say no objects of type OrbElem may
	 /// be constructed.   This is a good thing since OrbElem
	 /// doesn't even provide methods to load its' members.
	 /// Only its' descendents may be instantiated.
      virtual OrbElem* clone() const = 0;

         /**
          * Returns true if the time, ct, is within the period of validity of this OrbElem object.
          * @throw Invalid Request if the required data has not been stored.
          */
      virtual bool isValid(const CommonTime& ct) const throw(InvalidRequest);

	 /**
          *   Return true if orbit data have been loaded.
          *   Returns false if the object has been instantiated,
          *   but no data have been loaded.
          */
      virtual bool dataLoaded( ) const;

      virtual std::string getName() const = 0;

      virtual std::string getNameLong() const = 0;

         /** This function returns the health status of the SV.
          * @throw Invalid Request if the required data has not been stored.
          */
      bool isHealthy() const throw(gpstk::InvalidRequest);

         /** Compute the satellite clock bias (sec) at the given time
          *  @throw Invalid Request if the required data has not been stored.
          */
      double svClockBias(const CommonTime& t) const throw(gpstk::InvalidRequest);

         /** Compute the satellite clock bias (meters) at the given time
          *  @throw Invalid Request if the required data has not been stored.
          */
      double svClockBiasM(const CommonTime& t) const throw(gpstk::InvalidRequest);

         /** Compute the satellite clock drift (sec/sec) at the given time
          *  @throw Invalid Request if the required data has not been stored.
          */
      double svClockDrift(const CommonTime& t) const throw(gpstk::InvalidRequest);


         /** Compute satellite position at the given time
          * using this orbit data.
          * @throw Invalid Request if the required data has not been stored.
          */
      Xvt svXvt(const CommonTime& t) const throw(gpstk::InvalidRequest);

         /** Compute satellite relativity correction (sec) at the given time
          *  @throw Invalid Request if the required data has not been stored.
          */
      double svRelativity(const CommonTime& t) const throw( gpstk::InvalidRequest );


         /** adjustBeginningValidity is provided to support
          *  GPSOrbElemStore::rationalize( ).  It rounds the
          *  beginning time of validity back to the nominal
          *  beginning time in the case of a set of elements that
          *  are the second set following an upload.  It should not
          *  be used other than by GPSOrbElemStore::rationalize().
          *
          *  Since the means of determining validity varies between
          *  message format types, this function is pure virtual here
          *  and implemented in descendents.
          */
      virtual void adjustBeginningValidity() = 0;

         /** Output the contents of this orbit data to the given stream.
          * @throw Invalid Request if the required data has not been stored.
          */

      static void shortcut(std::ostream & os, const long HOW);

      static void timeDisplay(std::ostream & os, const CommonTime& t);

      virtual void dumpTerse(std::ostream& s = std::cout) const
         throw( InvalidRequest ) = 0;

      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      virtual void dumpFooter(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      virtual void dump(std::ostream& s = std::cout) const
	 throw( InvalidRequest );

         /// Overhead information
         //@{
      bool    dataLoadedFlag;  /**< True if data is present, False otherwise */
      SatID   satID;	       /**< Define satellite system and specific SV */
      ObsID   obsID;           /**< Defines carrier and tracking code */
      CommonTime ctToe;        /**< Orbit epoch in commontime format */
      bool    healthy;         /**< SV health (healthy=true, other=false */
              //@}

	 /// Harmonic perturbations
         //@{
      double   Cuc;           /**< Cosine latitude (rad) */
      double   Cus;           /**< Sine latitude (rad) */
      double   Crc;           /**< Cosine radius (m) */
      double   Crs;           /**< Sine radius (m) */
      double   Cic;           /**< Cosine inclination (rad) */
      double   Cis;           /**< Sine inclination (rad) */
         //@}

         /// Major orbit parameters
         //@{
      double   M0;            /**< Mean anomaly (rad) */
      double   dn;            /**< Correction to mean motion (rad/sec) */
      double   dndot;	      /**< Rate of correction to mean motion (rad/sec/sec) */
      double   ecc;           /**< Eccentricity */
      double   A;             /**< Semi-major axis (m) */
      double   Adot;          /**< Rate of semi-major axis (m/sec) */
      double   OMEGA0;        /**< Rt ascension of ascending node (rad) */
      double   i0;            /**< Inclination (rad) */
      double   w;             /**< Argument of perigee (rad) */
      double   OMEGAdot;      /**< Rate of Rt ascension (rad/sec) */
      double   idot;          /**< Rate of inclination angle (rad/sec) */
         //@}

         /// Clock information
         //@{
      CommonTime ctToc;	    /**< Clock Epoch in commontime format */
      double af0;           /**< SV clock error (sec) */
      double af1;           /**< SV clock drift (sec/sec) */
      double af2;           /**< SV clock drift rate (sec/sec**2) */
         //@}

         // Fit Interval Definition
         // The beginning and end of validity are derived quantities that specify
         // the bounds between which the data in OrbElem are valid.
         // IS-GPS-200, 705, and -800 are not specific regarding the
         // inclusion of the boundary conditions, but it is recommended
         // that algorithms that consider validity should treat the
         // bounds as (beginValid, endValid]. This is because beginValid
         // is tied to the beginning of the first transmission of the data
         // and the data will require some seconds (at least 18 seconds
         // in the case of legacy GPS navigation message data)  to be
         // transmitted. (See the appropriate loadData( ) function in
         // the relevant OrbElem descendents for details on how beginValid
         // and endValid are derived.

         //@{
      CommonTime beginValid;    /**< Time at beginning of validity */
      CommonTime endValid;      /**< Time at end of fit validity */



   }; // end class OrbElem

   //@}

//NB this was removed since it was never defined
   // std::ostream& operator<<(std::ostream& s,
   //                                    const OrbElem& eph);

} // end namespace

#endif // GPSTK_ORBELEM_HPP
