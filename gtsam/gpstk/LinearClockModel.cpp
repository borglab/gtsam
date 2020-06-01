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
 * @file LinearClockModel.cpp
 * This model is just a moving average of clock models over time. It
 * really is quite bad and shouldn't be used at this time. Be warned!
 */

#include <math.h>
#include "Stats.hpp"

#include "LinearClockModel.hpp"

namespace gpstk
{
   using namespace std;

   void LinearClockModel::reset() throw()
   {
      startTime = gpstk::CommonTime::END_OF_TIME;
      endTime = gpstk::CommonTime::BEGINNING_OF_TIME;
      clockObs.clear();
      prnStatus.clear();
      clockModel.Reset();
      tossCount=0;
   }

   void LinearClockModel::addEpoch(const ORDEpoch& oe)
      throw(gpstk::InvalidValue)
   {
      ORDEpoch::ORDMap::const_iterator itr;
      const gpstk::CommonTime t=oe.time;
      
      // Start off by getting an estimate of this epoch's clock
      // note that this also sets the prn status map
      gpstk::Stats<double> stat = simpleOrdClock(oe);
      SvStatusMap& statusMap = prnStatus[t];
      statusMap = status;

      double mean;
      if (clockModel.N()==0)
      {
         double clkc = stat.Average();
#pragma unused(clkc)
         startTime = endTime = baseTime = t;
         tossCount = 0;
      }

      const double deltaT = t-baseTime;

      if (t<startTime)
         startTime=t;
      if (t>endTime)
         endTime=t;

      if (clockModel.N()>24)
         mean = clockModel.Slope()*deltaT + clockModel.Intercept();
      else
         mean = stat.Average();

      if (std::abs(stat.Average() - mean) > 20)
      {
         cout << t
              << " slope=" << setw(12) << clockModel.Slope()
              << ", intercept=" << setw(8) << clockModel.Intercept()
              << ", est=" << setw(8) << clockModel.Slope()*deltaT + clockModel.Intercept()
              << ", N=" << setw(6) << clockModel.N()
              << ", stdev=" << setw(6) << clockModel.StdDevY()
              << endl;
         tossCount++;
         if (tossCount>5)
         {
            reset();
            cout << "Reseting model" << endl;
         }
      }
      else
      {
         tossCount=0;
         for (itr = oe.ords.begin(); itr != oe.ords.end(); itr++)
            if (statusMap[itr->second.getSvID()] == USED)
            {
               const double ord = itr->second.getORD();
               clockModel.Add(deltaT, ord);
               std::pair<const double,double> o(deltaT, ord);
               clockObs.insert(o);
            }
      }

      std::multimap<double,double>::iterator i1,i2;
      i1 = clockObs.begin();
      while (i1!=clockObs.end())
      {
         i2=i1;
         i1++;
         double dt = i2->first;
         double ord = i2->second;
         if ((deltaT - dt)>1800)
         {
            clockObs.erase(i2);
            clockModel.Subtract(dt, ord);
         }
         else
            break;
      }
   }

   void LinearClockModel::dump(std::ostream& s, short detail) const throw()
   {
      s << "base: " << baseTime
        << ", start: " << startTime
        << ", end: " << endTime
        << endl
        << "Clock: est(end)=" << getOffset(endTime)
        << ", n=" << clockModel.N()
        << ", b=" << clockModel.Intercept()
        << ", m=" << clockModel.Slope()
        << ", sigma=" << clockModel.StdDevY()
        << ", r=" << clockModel.Correlation()
        << endl;

      if (detail>0)
      {
         s << "min elev: " << elvmask
           << ", max sigma: " << sigmam
           << endl;

         map<CommonTime,SvStatusMap>::const_iterator e = prnStatus.find(endTime);
         const SvStatusMap& statusMap = e->second;
         SvStatusMap::const_iterator i;
         for ( i=statusMap.begin(); i!= statusMap.end(); i++)
            s << i->first << "/" << i->second << " ";
         s << endl;
      }
   }
}
