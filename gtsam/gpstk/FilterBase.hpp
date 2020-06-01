#pragma ident "$Id$"

/**
 * @file FilterBase.hpp
 * Abstract base class for filter algorithms.
 */

#ifndef GPSTK_FILTERBASE_HPP
#define GPSTK_FILTERBASE_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2011
//
//============================================================================



#include "Exception.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      /// @ingroup math
      //@{

      /**
       * Abstract base class for filter algorithms.
       */
   class FilterBase
   {
   public:


         /// Implicit constructor
      FilterBase() : valid(false) {};


         /** Abstract method. Returns result.
          *
          * @param input      Input data.
          */
      virtual double Compute( double input ) = 0;


         /// Abstract method. Resets filter.
      virtual void Reset(void) = 0;


         /// Returns validity of results
      bool isValid(void)
      { return valid; }


         /// Destructor
      virtual ~FilterBase() {};


   protected:


        bool valid;         // true only if results are valid


   }; // End of class 'FilterBase'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_FILTERBASE_HPP
