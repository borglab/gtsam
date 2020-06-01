#pragma ident "$Id$"

/**
 * @file ARSimple.hpp
 * 
 */

#ifndef GPSTK_ARSIMPLE_HPP
#define GPSTK_ARSIMPLE_HPP

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

#include "ARBase.hpp"

namespace gpstk
{
      /** This class resolve integer ambiguity by simply round  the float 
       *  ambiguities to the nearest integers.
       */
   class ARSimple   
   {
   public:
      
         /// Default constructor
      ARSimple(){}      
      
         /// Integer Ambiguity Resolution method
      virtual Vector<double> resolveIntegerAmbiguity( 
                                               const Vector<double>& ambFloat, 
                                               const Matrix<double>& ambCov )
         throw(ARException);
      
         /// Destructor
      virtual ~ARSimple(){}
      
   protected:
      
         
   };   // End of class 'ARSimple'
   
}   // End of namespace gpstk


#endif  //GPSTK_ARSIMPLE_HPP

