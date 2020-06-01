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

#ifndef EPOCHCLOCKMODEL_HPP
#define EPOCHCLOCKMODEL_HPP

#include <map>
#include "Exception.hpp"
#include "gps_constants.hpp"

#include "ObsClockModel.hpp"
#include "ORDEpoch.hpp"

/**
 * @file EpochClockModel.hpp
 * Finally a concrete class. This model just uses an epoch of ORDs to determine
 * the clock offset at that point in time.
 */

namespace gpstk
{
   class EpochClockModel : public ObsClockModel
   {
   public:
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
      EpochClockModel(double sigma = 2,
                      double elmask = 0, 
                      SvMode mode = ALWAYS)
         : ObsClockModel(sigma, elmask, mode), valid(false), clkc(0){}
#pragma clang diagnostic pop
      virtual double getOffset(const gpstk::CommonTime& t) const
         throw(gpstk::InvalidArgumentException) 
      {
         if (t!=time)
         {
            gpstk::InvalidArgumentException e;
            GPSTK_THROW(e);
         }
         return clkc;
      };

      virtual bool isOffsetValid(const gpstk::CommonTime& t) const 
         throw(gpstk::InvalidArgumentException)
      {
         if (t!=time) 
         {
            gpstk::InvalidArgumentException e;
            GPSTK_THROW(e);
         }
         return valid;
      };


      // An unchecked accessor for programs that don't need the generic
      // interface
      double getOffset() const
         throw() {return clkc;};

      bool isOffsetValid() const 
         throw() {return valid;};

      virtual void addEpoch(const ORDEpoch& oe) throw(gpstk::InvalidValue)
      {
         gpstk::Stats<double> stat = simpleOrdClock(oe);
         clkc = stat.Average();
         valid = stat.N() >=  3; /// we need at least three to have a real avg
         time = oe.time;
      }

   private:
      gpstk::CommonTime time;   ///< The time of this offset
      double clkc;           ///< clock bias value (same units as residuals)
      bool valid;            ///< flag indicating clock bias statistical validity
   };
}
#endif
