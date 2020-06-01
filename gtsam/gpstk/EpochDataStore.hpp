#pragma ident "$Id$"

/**
* @file EpochDataStore.hpp
* Class to handle interpolatable time serial data 
*/

#ifndef GPSTK_EPOCHDATASTORE_HPP
#define GPSTK_EPOCHDATASTORE_HPP


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


#include <iostream>
#include <string>
#include <set>
#include <map>
#include "CommonTime.hpp"


namespace gpstk
{
   
      /** @addtogroup General Data Tool */
      //@{

      /** Class to handle interpolatable time serial data 
       * 
       */
   class EpochDataStore
   {
   public:

         /// Handy type definition
      typedef std::set<CommonTime> EpochList;
      typedef std::map<CommonTime, std::vector<double> > EpochData;
      

         /// Default constructor
      EpochDataStore()
         : initialTime(CommonTime::END_OF_TIME),
           finalTime(CommonTime::BEGINNING_OF_TIME),
           interPoints(10)
      {}

      EpochDataStore(int interpolationPoints)
         : initialTime(CommonTime::END_OF_TIME),
         finalTime(CommonTime::BEGINNING_OF_TIME),
         interPoints(interpolationPoints)
      {}

         /// Default deconstructor
      virtual ~EpochDataStore()
      { allData.clear(); }
         
         /// get epoch list stored in this object
      EpochList epochList();

      bool isEpochExist(CommonTime t)
      { return (allData.find(t) != allData.end()) ? true : false ; }
      
         /// clear the all the data
      void clear()
      { allData.clear(); }

         /** Edit the dataset, removing data outside the indicated time
          *  interval.
          *
          * @param[in] tmin defines the beginning of the time interval
          * @param[in] tmax defines the end of the time interval
          */
      virtual void edit(CommonTime tmin,
                        CommonTime tmax = CommonTime::END_OF_TIME);

         /// return the number of entries in the store
      size_t size(void)
      { return allData.size(); }

         /** Determine the earliest time stored in the object 
          *
          * @return The initial time
          *
          */
      virtual CommonTime getInitialTime()
      { return initialTime; };


         /** Determine the latest time stored in the object
          *
          * @return The final time
          *
          */
      virtual CommonTime getFinalTime()
      { return finalTime; };


      EpochDataStore& setInterpolationPoints(const int& n)
      { interPoints = n; return (*this); }


   protected:

         /// Add to the store directly
      void addData(const CommonTime& time,const std::vector<double>& data);


         /** Get the Data at the given epoch and return it.
          *  @param t CommonTime at which to compute the EOPs.
          *  @return EarthOrientation EOPs at time t.
          *  @throw InvalidRequest if the epoch on either side of t
          *     cannot be found in the map.
          */
      std::vector<double> getData(const CommonTime& t) const
         throw(InvalidRequest);

      
         /// Object holding all the data for the vehicle
      EpochData allData;
         
         /// These give the overall span of time for which this object
         ///  contains data.
      CommonTime initialTime;
      CommonTime finalTime;
      
         /// Number of points to do Lagrange Interpolation, default is 10
      int interPoints;

   }; // End of class 'EpochDataStore'

      // @}

}  // End of namespace gpstk


#endif   // GPSTK_EPOCH_DATA_STORE_HPP






