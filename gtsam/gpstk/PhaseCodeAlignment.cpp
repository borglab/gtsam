//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011
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
 * @file PhaseCodeAlignment.cpp
 * This class aligns phase with code measurements.
 */

#include <gtsam/gpstk/PhaseCodeAlignment.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string PhaseCodeAlignment::getClassName() const
   { return "PhaseCodeAlignment"; }


      /* Common constructor
       *
       * @param phase            Phase TypeID.
       * @param code             Code TypeID.
       * @param wavelength       Phase wavelength, in meters.
       * @param useArc           Whether satellite arcs will be used or not.
       */
   PhaseCodeAlignment::PhaseCodeAlignment( const TypeID& phase,
                                           const TypeID& code,
                                           const double wavelength,
                                           bool useArc )
      : phaseType(phase), codeType(code), useSatArcs(useArc),
        watchCSFlag(TypeID::CSL1)
   {

         // Set the wavelength
      setPhaseWavelength(wavelength);

   }  // End of 'PhaseCodeAlignment::PhaseCodeAlignment()'



      /* Method to set the phase wavelength to be used.
       *
       * @param wavelength       Phase wavelength, in meters.
       */
   PhaseCodeAlignment& PhaseCodeAlignment::setPhaseWavelength(double wavelength)
   {

         // Check that wavelength is bigger than zero
      if (wavelength > 0.0)
      {
         phaseWavelength = wavelength;
      }
      else
      {
         phaseWavelength = 0.1069533781421467;   // Be default, LC wavelength
      }

      return (*this);

   }  // End of 'PhaseCodeAlignment::setPhaseWavelength()'



      /* Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& PhaseCodeAlignment::Process( const CommonTime& epoch,
                                           satTypeValueMap& gData )
      throw(ProcessingException)
   {

      try
      {

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         for( satTypeValueMap::iterator it = gData.begin();
              it != gData.end();
              ++it )
         {

               // Check if satellite currently has entries
            std::map<SatID, alignData>::const_iterator itDat(
                                                svData.find( (*it).first ) );
            if( itDat == svData.end() )
            {

                  // If it doesn't have an entry, insert one
               alignData aData;

               svData[ (*it).first ] = aData;

            }


               // Place to store if there was a cycle slip. False by default
            bool csflag(false);


               // Check if we want to use satellite arcs of cycle slip flags
            if(useSatArcs)
            {

               double arcN(0.0);

               try
               {

                     // Try to extract the satellite arc value
                  arcN = (*it).second(TypeID::satArc);

               }
               catch(...)
               {

                     // If satellite arc is missing, then schedule this
                     // satellite for removal
                  satRejectedSet.insert( (*it).first );

                  continue;

               }


                  // Check if satellite arc has changed
               if( svData[(*it).first].arcNumber != arcN )
               {

                     // Set flag
                  csflag = true;

                     // Update satellite arc information
                  svData[(*it).first].arcNumber = arcN;
               }

            }  // End of first part of 'if(useSatArcs)'
            else
            {

               double flag(0.0);

               try
               {

                     // Try to extract the CS flag value
                  flag = (*it).second(watchCSFlag);

               }
               catch(...)
               {

                     // If flag is missing, then schedule this satellite
                     // for removal
                  satRejectedSet.insert( (*it).first );

                  continue;

               }

                  // Check if there was a cycle slip
               if( flag > 0.0)
               {
                     // Set flag
                  csflag = true;
               }

            }  // End of second part of 'if(useSatArcs)...'


               // If there was an arc change or cycle slip, let's
               // compute the new offset
            if(csflag)
            {

                  // Compute difference between code and phase measurements
               double diff( (*it).second(codeType) - (*it).second(phaseType) );

                  // Convert 'diff' to cycles
               diff = diff/phaseWavelength;

                  // Convert 'diff' to an INTEGER number of cycles
               diff = std::floor(diff);

                  // The new offset is the INTEGER number of cycles, in meters
               svData[(*it).first].offset = diff * phaseWavelength;

            }

               // Let's align the phase measurement using the
               // corresponding offset
            (*it).second[phaseType] = (*it).second[phaseType]
                                      + svData[(*it).first].offset;

         }

            // Remove satellites with missing data
         gData.removeSatID(satRejectedSet);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of 'PhaseCodeAlignment::Process()'



      /* Returns a gnnsSatTypeValue object, adding the new data generated
       *  when calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssSatTypeValue& PhaseCodeAlignment::Process(gnssSatTypeValue& gData)
      throw(ProcessingException)
   {

      try
      {

         Process(gData.header.epoch, gData.body);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of 'PhaseCodeAlignment::Process()'



      /* Returns a gnnsRinex object, adding the new data generated when
       *  calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& PhaseCodeAlignment::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

         Process(gData.header.epoch, gData.body);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of 'PhaseCodeAlignment::Process()'


} // End of namespace gpstk
