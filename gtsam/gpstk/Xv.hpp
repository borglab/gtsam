#pragma ident "$Id$"



/**
 * @file Xv.hpp
 * Position and velocity, representation as Triples
 */

#ifndef GPSTK_XV_HPP
#define GPSTK_XV_HPP

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
//  Copyright 2004, The University of Texas at Austin
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






#include <iostream>
#include "Triple.hpp"

namespace gpstk
{
    /** @addtogroup geodeticgroup */
    //@{

      /// An Earth-Centered, Earth-Fixed position/velocity/clock representation
   class Xv
   {
   public:
         /// Default constructor
      Xv() { }

      Triple x;         ///< SV position (x,y,z). Earth-fixed. meters
      Triple v;       ///< SV velocity. Earth-fixed, including rotation. meters/sec

   }; 

   //@}

}

/**
 * Output operator for Xv
 * @param s output stream to which xv is sent
 * @param xv Xv that is sent to s
 */
inline std::ostream& operator<<( std::ostream& s, 
                          const gpstk::Xv& xv )
{
   s << "x:" << xv.x
     << ", v:" << xv.v;
   return s;
}

#endif
