#pragma ident "$Id$"

/**
 * @file ExtractPC.hpp
 * This class eases PC combination data extraction from a Rinex3ObsData object.
 */

#ifndef GPSTK_EXTRACTPC_HPP
#define GPSTK_EXTRACTPC_HPP

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
//  Dagoberto Salazar - gAGE. 2006, 2012
//
//============================================================================


#include "ExtractCombinationData.hpp"
#include "GNSSconstants.hpp"


namespace gpstk
{

      /** @addtogroup RinexObs */
      //@{


      /// This class eases PC combination data extraction from
      /// a Rinex3ObsData object.
   class ExtractPC : public ExtractCombinationData
   {
   public:

         /// Default constructor
      ExtractPC()
         : typeObs1("P1"), typeObs2("P2")
      { valid = false; checkData = true; };


         /** Compute the PC observation from a Rinex3ObsData object.
          *
          * @param rinexData  The Rinex data set holding the observations.
          * @param hdr        RINEX Observation Header for current RINEX file.
          *
          * @return
          *  Number of satellites with PC combination data available
          */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
      virtual int getData( const Rinex3ObsData& rinexData,
                           const Rinex3ObsHeader& hdr )
         throw(InvalidRequest)
      {

         return ExtractCombinationData::getData( rinexData,
                                                 typeObs1,
                                                 typeObs2,
                                                 hdr );

      }; // End of method 'ExtractPC::getData()'

#pragma clang diagnostic pop
         /// Some Rinex data files provide C1 instead of P1. Use this method
         /// in those cases.
      virtual ExtractPC& useC1()
      { typeObs1 = "C1"; return (*this); };


         /// Destructor
      virtual ~ExtractPC() {};


   protected:


         /// Compute the combination of observables.
      virtual double getCombination( double obs1, double obs2 )
         throw(InvalidRequest)
      {
         return ( (GAMMA_GPS*obs1 - obs2)/(GAMMA_GPS - 1.0) );
      };

   private:

      std::string typeObs1;
      std::string typeObs2;


   }; // End of class 'ExtractPC'


      //@}

}  // End of namespace gpstk

#endif   // GPSTK_EXTRACTPC_HPP
