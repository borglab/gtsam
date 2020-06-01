#pragma ident "$Id$"

/**
 * @file ARMLambda.hpp
 * 
 */

#ifndef GPSTK_ARMLAMBDA_HPP
#define GPSTK_ARMLAMBDA_HPP

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
//  Wei Yan - Chinese Academy of Sciences . 2011
//
//============================================================================

#include "ARLambda.hpp"

namespace gpstk
{
      /** This class resolve integer ambiguity by the Modified LAMBDA method.
       *
       * The algorithm was described by:
       *
       *   X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
       *   integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
       *
       */
   class ARMLambda : public ARLambda  
   {
   public:
      
         /// Default constructor
      ARMLambda(){}      
      

         /// Destractor
      virtual ~ARMLambda(){}
      
   protected:

         /// modified lambda (mlambda) search
      virtual int search( const Matrix<double>& L, 
                          const Vector<double>& D, 
                          const Vector<double>& zs, 
                          Matrix<double>& zn, 
                          Vector<double>& s, 
                          const int& m = 2 );
         
   };   // End of class 'ARMLambda'
   

}   // End of namespace gpstk


#endif  //GPSTK_ARMLAMBDA_HPP

