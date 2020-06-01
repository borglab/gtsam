#pragma ident "$Id$"

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

/**
 * @file EngAlmanac.hpp
 * Almanac data encapsulated in engineering terms
 */

#ifndef GPSTK_ENGALMANAC_HPP
#define GPSTK_ENGALMANAC_HPP

#include "Exception.hpp"
#include "EngNav.hpp"
#include "AlmOrbit.hpp"
#include "Xvt.hpp"
#include "StringUtils.hpp"

namespace gpstk
{

      /// Exception - requested almanac page that wasn't present.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SVNotPresentException, gpstk::InvalidRequest);


   /** @addtogroup ephemcalc */
   //@{
      /**
       * Almanac information for the GPS constellation.  This class
       * encapsulates the almanac navigation message (subframes 4 & 5)
       * and provides functions to decode the as-broadcast almanac.
       * It is possible for an EngAlmanac to not contain a complete
       * set of pages.
       */
   class EngAlmanac : public EngNav
   {
   public:

         /// Map PRN to bits (e.g. health bits).
      typedef std::map<short, unsigned char, std::less<short> > SVBitsMap;

         /// Default constructor, blank almanac.
      EngAlmanac() throw();

         /// Destructor
      virtual ~EngAlmanac() {}

         /**
          * Store a subframe in this object.
          * @param subframe ten word navigation subframe stored in the
          * 30 least-significant bits of each array index.
          * @param gpsWeek full GPS week number.
          * @return true if successful.
          * @throw InvalidParameter if subframe is valid but not subframe 4-5.
          */
      bool addSubframe(const long subframe[10], const int gpsWeek)
         throw(gpstk::InvalidParameter);

         /** This function returns true if data is available for a given
          * PRN.  This data is accessed by the below accesser methods
          */
      bool isData(SatID sat) const throw();

         /** This function returns the value of the eccentricity for
          * the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getEcc(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the offset of the
          * inclination from 54 degrees in radians for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getIOffset(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the rate of the right
          * ascension of the ascending node in radians/second for the
          * given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getOmegadot(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the health of the given
          * PRN from the general pages in the almanac.  It return the
          * shortened 6 bit health that is in those pages.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      short get6bitHealth(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the health of the given
          * PRN from the PRN specific page which might not be present.
          * This is the full 8 bit health
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      short getSVHealth(SatID sat) const throw(SVNotPresentException);


         /** This function returns the four-bit A/S-flag and configuration
          * bits for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      short getSVConfig(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the square root of the
          * semi-major axis in square root of meters for the given
          * PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getAhalf(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the semi-major axis in
          * meters for the specified PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getA(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the right ascension of
          * the ascending node in radians for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getOmega0(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the argument of perigee
          * in radians for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getW(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the mean anomaly in
          * radians for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getM0(SatID sat) const throw(SVNotPresentException);

         /** This function returns the SV clock error in seconds for
          * the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getAf0(SatID sat) const throw(SVNotPresentException);

         /** This function returns the SV clock drift in
          * seconds/seconds for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getAf1(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the time of the almanac
          * (from page 51) in GPS seconds of week.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getToa() const throw();

         /** This function returns the value of the time of the almanac
          * in GPS seconds of week for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getToa(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the transmit time for
          * this almanac data in seconds of week for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      double getXmitTime(SatID sat) const throw(SVNotPresentException);

         /** This function returns the value of the week of the page
          * transmission for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      short getFullWeek(SatID sat) const throw(SVNotPresentException);

         /**
          * Get the ionospheric parameters.
          * @throw InvalidRequest if the almanac page isn't present
          */
      void getIon(double a[4], double b[4]) const
         throw(InvalidRequest);

         /**
          * Get the UTC offset parameters.
          * @throw InvalidRequest if the almanac page isn't present
          */
      void getUTC(double& a0, double& a1, double& deltaTLS, long& tot,
                  int& WNt, int& WNLSF, int& DN, double& deltaTLSF) const
         throw(InvalidRequest);

         /** This function gets the week number for the almanac stored
          * in this object.  It is replaced when an almanac is
          * converted to engineering units with the almWeek from the
          * data.  It also is replaced by the week number in the FIC
          * data (if it is non zero) when addSF is used to add FIC
          * data to the almanac.  This is a full GPS week number (ie >
          * 10 bits)
          */
      short getAlmWeek() const throw();

         /** This function returns an object containing all of the
          * almanac orbit elements for the given PRN.
          * @throw SVNotPresentException if almanac page for the given
          * PRN isn't present.
          */
      AlmOrbit getAlmOrbElem(SatID sat) const
         throw(SVNotPresentException);

         /** This function returns an object containing all of the
          * almanac orbit elements.
          */
      AlmOrbits getAlmOrbElems() const
      { return almPRN; }

         /** Compute satellite velocity/position at the given time
          * using this almanac.
          * @param sat SatID of satellite to get velocity/position of.
          * @param t time at which to compute SV position.
          * @throw InvalidRequest if a required subframe has not been stored.
          */
      Xvt svXvt(SatID sat, const CommonTime& t) const
         throw(SVNotPresentException);

         /// \deprecated use the SatID version
      Xvt svXvt(short prn, const CommonTime& t) const
         throw(SVNotPresentException)
         { SatID sat(prn,SatID::systemGPS); return svXvt(sat,t); }

      void dump(std::ostream& s = std::cout, bool checkFlag=true) const;

      bool check(std::ostream& s) const;

   protected:
         /** This function is used to make sure data is present before
          * accessing it.
          */
      void checkSVHere(SatID sat) const throw(SVNotPresentException);


         /** ionosphere parameters */
         //@{
      double alpha[4];
      double beta[4];
         //@}

         /** UTC Parameters */
         //@{
      double A0;                   /**< Bias term of difference polynomial */
      double A1;                   /**< Drift term of difference polynomial */
      double dt_ls;                /**< time increment due to leap seconds */
      double dt_lsf;               /**< scheduled future time increment due to
                                      leap seconds */
      long t_ot;                   /**< reference time */
      long t_oa;                   /**< Toa from page id 51 (subframe 5,
                                      pg 25) */
      int wn_t;                    /**< reference week of current leap
                                      second */
      int wn_lsf;                  /**< week number of last/next leap
                                      second */
      short alm_wk;                /**< GPS Week of the Almanac from the last
                                      page of orbital data */
      unsigned char dn;            /**< reference day # of future leap
                                      second */
      SVBitsMap health;            /**< satellite health array */
      std::string special_msg;     /**< Special message from GPS */

      SVBitsMap SV_config;         /**< 4 bit anti-spoofing/SV config sats. */
         //@}

      AlmOrbits almPRN;
      bool haveUTC;

   private:
      bool operator==(const EngAlmanac&);
      bool operator!=(const EngAlmanac&);
      bool operator<(const EngAlmanac&);
      bool operator>(const EngAlmanac&);
   }; // class EngAlmanac


   std::ostream& operator<<(std::ostream& s, const EngAlmanac& alm);
   //@}

} // namespace

#endif
