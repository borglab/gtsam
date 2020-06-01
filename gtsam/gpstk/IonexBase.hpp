#pragma ident "$Id$"

/**
 * @file IonexBase.hpp
 * Base class for IONEX (IONosphere Map EXchange) file data
 */

#ifndef GPSTK_IONEXBASE_HPP
#define GPSTK_IONEXBASE_HPP


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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008
//
//============================================================================


#include "FFData.hpp"

namespace gpstk
{


      /** @defgroup IonosphereMaps IONEX format file I/O */
      //@{

      /** This class does not served any purpose other than to make readable
       *  inheritance diagrams.
       */
   class IonexBase : public FFData
   {
   public:

         /// Destructor per the coding standards
      virtual ~IonexBase() {};

   }; // End of class 'IonexBase'


      //@}


}  // End of namespace gpstk

#endif   // GPSTK_IONEXBASE_HPP
