#pragma ident "$Id$"

/**
* @file EpochDataStore.cpp
* Class to handle interpolatable time serial data 
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
//  Wei Yan - Chinese Academy of Sciences . 2009, 2010, 2011
//
//============================================================================


#include "EpochDataStore.hpp"
#include "MiscMath.hpp"

namespace gpstk
{
   using namespace std;

      // get epoch list stored in this object
   EpochDataStore::EpochList EpochDataStore::epochList()
   {
      EpochList epochList;
      for(EpochData::const_iterator it = allData.begin();
         it != allData.end();
         ++it)
      {
         epochList.insert(it->first);
      }
      
      return epochList;
   }
   
      /* Edit the dataset, removing data outside the indicated time
       *  interval.
       *
       * @param[in] tmin defines the beginning of the time interval
       * @param[in] tmax defines the end of the time interval
       */
   void EpochDataStore::edit( CommonTime tmin, CommonTime tmax  )
   {
      if(tmin > tmax) 
      {
         CommonTime m= tmin;
         tmin = tmax;
         tmax = m;
      }

      if(tmin > finalTime) return;
      if(tmax < initialTime) return;

      EpochData::iterator it = allData.lower_bound(tmin);
      if(it != allData.begin())
      {
         allData.erase(allData.begin(), it);
      }

      it = allData.upper_bound(tmax);
      if(it != allData.end())
      {
         allData.erase(it, allData.end());
      }

      it = allData.begin();
      if(it == allData.end())
      {
         initialTime = CommonTime::END_OF_TIME;
      }
      else
      {
         initialTime = it->first;
      }

      it = allData.end();
      if(--it == allData.end())
      {
         finalTime = CommonTime::BEGINNING_OF_TIME;
      }
      else 
      {
         finalTime = it->first;
      }

   } // End of method 'EpochDataStore::edit()'
   
   
      // Add to the store directly
   void EpochDataStore::addData(const CommonTime& time, 
                                const std::vector<double>& data)
   {
      
      allData[time] = data;

      if((initialTime == CommonTime::END_OF_TIME)    ||
         (finalTime == CommonTime::BEGINNING_OF_TIME) )
      {
         initialTime = finalTime = time;
      }
      else if(time < initialTime) 
      {
         initialTime = time;
      }
      else if(time > finalTime) 
      {
         finalTime = time;
      }

   }  // End of method 'EpochDataStore::addData()'
   
   
      /* Get the VehiclePVData at the given epoch and return it.
       *  @param t CommonTime at which to compute the EOPs.
       *  @return EarthOrientation EOPs at time t.
       *  @throw InvalidRequest if the epoch on either side of t
       *     cannot be found in the map.
       */
   std::vector<double> EpochDataStore::getData(const CommonTime& t) const
         throw(InvalidRequest)
   {
      // check the time
      if( (t < initialTime) || (t > finalTime))
      {
         InvalidRequest ire(string("Time tag (")
            + t.asString()
            + string(") not found within the store "));
         GPSTK_THROW(ire);
      }

      EpochData::const_iterator it = allData.find(t);
      if(it != allData.end())
      {
         return it->second;
      }
      
     
      const int half = ( interPoints + 1 ) / 2;

      it = allData.lower_bound(t);   // i points to first element with key >= t

      if(t > finalTime) 
      { 
         it = allData.end();
         it--;
      }
 
      EpochData::const_iterator its = it;
      EpochData::const_iterator ite = it;

      if(int(allData.size())> 2*half)
      {
         int ileft = half;
         for(int i = 0; i < half; i++)
         {
            if(its==allData.begin()) break;
            its--;
            ileft--;
         }


         int iright = half-1+ileft;
         for(int i = 0; i < (half-1+ileft); i++)
         {
            ite++;

            if(ite == allData.end())
            {
               ite--;
               break;
            }

            iright--;
         }

         int ileft2 = iright; 
         for(int i = 0; i < iright; i++)
         {
            if(its == allData.begin()) break;
            its--;
            ileft2--;
         }

         if(ileft2 > 0)
         {
            // the code never go here
            // just throw an exception

            InvalidRequest e("My God, it should never go here!!!");
            GPSTK_THROW(e);
         }
      }
      else
      {
         its = allData.begin();
         ite = allData.end();
         ite--;
      }
      
      const int N = its->second.size();

      std::vector<double> times;
      std::vector<std::vector<double> > datas(N);
      
      EpochData::const_iterator itrEnd = ite;
      itrEnd++;
      for(EpochData::const_iterator itr=its; itr!=itrEnd; itr++)
      {
         CommonTime time = itr->first;
         std::vector<double> vd = itr->second;

         times.push_back(itr->first - its->first);
         
         if( vd.size() != N)
         {
            // Exception
            InvalidRequest e("Size of the data vector doesn't match!");
            GPSTK_THROW(e);
            
         }
         
         for(int i = 0; i < N; i++)
         {
            datas[i].push_back( vd[i] );
         }
      }
      
      std::vector<double> vd(N, 0.0);

      double dt = t - its->first;
      
      for(int i = 0; i < N; i++)
      {
         vd[i] = SimpleLagrangeInterpolation(times,datas[i],dt);
      }

      return vd;
   
   }  // End of method 'EpochDataStore::getPVData()'


}  // End of namespace gpstk









