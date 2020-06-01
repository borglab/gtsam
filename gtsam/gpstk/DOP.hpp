#pragma ident "$Id$"

/**
 * @file DOP.hpp
 * Class encapsulating the computation of DOP.
 */

#ifndef DOP_GPSTK
#define DOP_GPSTK

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
#include "MiscMath.hpp"


namespace gpstk
{
    /// Thrown when some problem appeared when computing DOP
    /// @ingroup exceptiongroup
    NEW_EXCEPTION_CLASS(InvalidDOP, gpstk::Exception);


    /** @addtogroup GPSsolutions */
    //@{


    /** This class encapsulates the computation of DOP, given the Covariance matrix 
     *  of an equation system.
     */
    class DOP
    {
    public:
        /// Return validity of results
        bool isValid(void)
            { return valid; }

        /// Geometric Dilution of Precision
        double GDOP;

        /// Position Dilution of Precision
        double PDOP;

        /// Time Dilution of Precision
        double TDOP;

        /// Implicit constructor
        DOP() throw(InvalidDOP) { valid = false; };


        /** Compute the DOP values associated with the given Covariance Matrix
         * @param covarianceMatrix      Covariance matrix for the equation system
         *
         * @return
         *  0 if OK
         *  -1 if problems arose
         */
        virtual int Compute(const Matrix<double>& covarianceMatrix) throw(InvalidDOP);


        /// Destructor
        virtual ~DOP() {};


    protected:
        bool valid;         // true only if results are valid

   }; // end class SolverBase
   

   //@}
   
}

#endif
