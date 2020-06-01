#pragma ident "$Id$"

/**
 * @file DOP.cpp
 * Class encapsulating the computation of DOP.
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
//  Dagoberto Salazar - gAGE. 2006
//
//============================================================================


#include "DOP.hpp"


namespace gpstk
{

    // Compute the DOP values associated with the given Covariance Matrix
    // @param covarianceMatrix      Covariance matrix for the equation system
    //
    // @return
    //   0 if OK
    //  -1 if problems arose
    //
    int DOP::Compute(const Matrix<double>& covarianceMatrix) throw(InvalidDOP)
    {
        int covCol = (int) covarianceMatrix.cols();
        int covRow = (int) covarianceMatrix.rows();
        if (!(covRow==covCol)) {
            InvalidDOP e("covarianceMatrix is not square");
            GPSTK_THROW(e);
        }

        try { 
            GDOP = RSS(covarianceMatrix(0,0), covarianceMatrix(1,1), covarianceMatrix(2,2), covarianceMatrix(3,3));
            PDOP = RSS(covarianceMatrix(0,0), covarianceMatrix(1,1), covarianceMatrix(2,2));
            TDOP = SQRT(covarianceMatrix(3,3));
        }
        catch(...) {
            InvalidDOP e("Unable to compute RSS of covarianceMatrix values.");
            GPSTK_THROW(e);
        }


        // If everything is fine so far, then the results should be valid
        valid = true;

        return 0;

    }  // end DOP::Compute()


} // end namespace gpstk
