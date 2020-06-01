#pragma ident "$Id$"

/**
 * @file WeightBase.hpp
 * Abstract base class for algorithms assigning weights to satellites.
 */

#ifndef WEIGHT_BASE_GPSTK
#define WEIGHT_BASE_GPSTK

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
//  Dagoberto Salazar - gAGE. 2006
//
//============================================================================



#include "Exception.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"


namespace gpstk
{
    /// Thrown when some problem appeared when assigning weights to satellites
    /// @ingroup exceptiongroup
    NEW_EXCEPTION_CLASS(InvalidWeights, gpstk::Exception);


    /** @addtogroup GPSsolutions */
    //@{

    /**
     * Abstract base class for algorithms assigning weights to satellites.
     */
    class WeightBase
    {
    public:

        /// Destructor
        virtual ~WeightBase() {};


    protected:
        bool valid;         // true only if weights are valid

   }; // end class WeightBase
   

   //@}
   
}

#endif
