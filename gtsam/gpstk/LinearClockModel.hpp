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

#ifndef LINEARCLOCKMODEL_HPP
#define LINEARCLOCKMODEL_HPP

#include <map>

#include "Exception.hpp"

#include "ObsClockModel.hpp"
#include "ORDEpoch.hpp"

/**
 * @file LinearClockModel.hpp
 * This model is just a moving average of clock models over time. It
 * really is quite bad and shouldn't be used at this time. Be warned!
 */

namespace gpstk
{
   class LinearClockModel : public ObsClockModel
   {
   public:
      LinearClockModel(double sigma = 2, double elmask = 0, SvMode mode = ALWAYS)
         :ObsClockModel(sigma, elmask, mode) {reset();};

      virtual double getOffset(const gpstk::CommonTime& t) const 
         throw()
      {
         if (!isOffsetValid(t))
            return 0;
         else
            return clockModel.Slope()*(t-baseTime) + clockModel.Intercept();
      };

      virtual bool isOffsetValid(const gpstk::CommonTime& t) const throw()
      {return t >= startTime && t <= endTime && clockModel.N() > 1;};

      /// Add in the given ord to the clock model
      virtual void addEpoch(const ORDEpoch& oe) throw(gpstk::InvalidValue);

      /// Reset the accumulated statistics on the clock
      void reset() throw();

      void dump(std::ostream& s, short detail=1) const throw();

      friend std::ostream& operator<<(std::ostream& s, const LinearClockModel& r)
      { r.dump(s, 0); return s; };
      
   private:
      // x is time y is clock offset
      gpstk::TwoSampleStats<double> clockModel;

      gpstk::CommonTime startTime, endTime, baseTime;

      unsigned long tossCount;
   
      // This is were we store what SVs were used to compute the individual
      // clock observations
      std::map<gpstk::CommonTime, SvStatusMap> prnStatus;

      // This is a store of the clock observations that were added into the
      // clockModel object
      std::multimap<double,double> clockObs;
   };
   
}
#endif
