#pragma ident "$Id$"

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

/**
 * @file YumaHeader.hpp
 * Encapsulate Yuma Almanac header, including I/O
 */

#ifndef YUMAHEADER_HPP
#define YUMAHEADER_HPP

#include <vector>
#include <list>
#include <map>

#include "FFStream.hpp"
#include "AlmOrbit.hpp"
#include "YumaBase.hpp"
#include "StringUtils.hpp"

namespace gpstk
{
   /** @addtogroup Yuma */
   //@{

   /** 
    * This class does not really do anything.  It is here to conform to the
    * other file types, even though the Yuma file type 
    * does not have any header information.
    *
    * @sa tests/Yuma for examples
    * @sa YumaStream.
    * @sa YumaData for more information on writing Yuma files.
    */
   class YumaHeader : public YumaBase
   {
   public:
      /// Constructor.
      YumaHeader() {}

      /// Destructor
      virtual ~YumaHeader() {}
      

      virtual void dump(std::ostream& s) const {};
      
      //! This class is a "header" so this function always returns "true". 
      virtual bool isHeader() const {return true;}

   protected:      
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException)
      {}
  
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException)
      {}
      
   }; // class YumaHeader

   //@}

} // namespace

#endif
