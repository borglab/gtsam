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
 * @file ProcessingClass.hpp
 * This is an abstract base class for objects processing GNSS Data Structures.
 */

#ifndef PROCESSINGCLASS_HPP
#define PROCESSINGCLASS_HPP

#include <gtsam/gpstk/StringUtils.hpp>
#include <gtsam/gpstk/DataStructures.hpp>


namespace gpstk
{

      /// Thrown when there is a problem processing GDS data.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(ProcessingException, gpstk::Exception);

    /** @addtogroup GPSsolutions */
    //@{


      /** This is an abstract base class for objects processing GNSS Data
       *  Structures (GDS).
       *
       * Children of this class are meant to be used together with GNSS data
       * structures objects found in "DataStructures" class, processing and
       * transforming them.
       *
       * A typical way to use a derived class follows:
       *
       * @code
       *   RinexObsStream rin("ebre0300.02o");
       *
       *   gnssRinex gRin;        // This is a GDS object
       *   ComputeLC getLC;       // ComputeLC is a child from ProcessingClass
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> getLC;      // getLC objects 'process' data inside gRin
       *   }
       * @endcode
       *
       * All children from ProcessingClass must implement the following methods:
       *
       * - Process(): These methods will be in charge of doing the real
       *   processing on the data.
       * - getIndex(): This method should return an unique index identifying
       *   the object.
       * - getClassName(): This method should return a string identifying the
       *   class the object belongs to.
       *
       */
   class ProcessingClass
   {
   public:


         /** Abstract method. It returns a gnnsSatTypeValue object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData) = 0;


         /** Abstract method. It returns a gnnsRinex object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData) = 0;


         /// Abstract method. It returns a string identifying the class the
         /// object belongs to.
      virtual std::string getClassName(void) const = 0;


         /// Destructor
      virtual ~ProcessingClass() {};



   }; // End of class 'ProcessingClass'


      /// Input operator from gnssSatTypeValue to ProcessingClass.
   inline gnssSatTypeValue& operator>>( gnssSatTypeValue& gData,
                                        ProcessingClass& procClass )
   { procClass.Process(gData); return gData; }


      /// Input operator from gnssRinex to ProcessingClass.
   inline gnssRinex& operator>>( gnssRinex& gData,
                                 ProcessingClass& procClass )
   { procClass.Process(gData); return gData; }


   //@}

}  // End of namespace gpstk

#endif   // PROCESSINGCLASS_HPP
