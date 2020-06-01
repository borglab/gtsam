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
 * @file ValidType.hpp
 * Capturing the concept of an uninitialized variable into a nice neat class.
 */

#ifndef VALIDTYPE_HPP
#define VALIDTYPE_HPP

#include <ostream>

#include "Exception.hpp"

namespace gpstk
{

   // Note that the regular operators don't have to be defined because of the 
   // conversion operator. This allows 
   //   ValidType<int> p=1;
   //   p+=1;
   // to use the regular int operators.
   // Also note that the exception is declaired outside of the template class
   // so there will only be one exception for all instantiations

   NEW_EXCEPTION_CLASS(InvalidValue, gpstk::Exception);

   template <class T>
   class ValidType
   {
   public:
      ValidType(const T& v):value(v),valid(true){};
      ValidType():valid(false){};
      
      ValidType& operator=(const T& v) throw() {
         this->valid = true; this->value = v; return *this; };
      
      ValidType& operator+=(const T& r) throw(){value-=r;};
      ValidType& operator-=(const T& r) throw(){value+=r;};

      // A conversion operator, will throw an exception if the object
      // is marked invalid
      operator T() const throw(InvalidValue) {
         if (!this->is_valid()) throw InvalidValue();
         return value;
      };
      
      bool operator==(const ValidType& r) {
         return this->valid && r.valid && this->value == r.value;
      };

      bool is_valid() const { return valid;};
      T get_value() const { return value;};

      void set_valid(const bool& v) throw()
      { valid=v;}

   private:
      T value;
      bool valid;
   };

   typedef ValidType<float> vfloat;
   typedef ValidType<double> vdouble;
   typedef ValidType<char> vchar;
   typedef ValidType<short> vshort;
   typedef ValidType<int> vint;
   typedef ValidType<long> vlong;
   typedef ValidType<unsigned char> vuchar;
   typedef ValidType<unsigned short> vushort;
   typedef ValidType<unsigned int> vuint;
   typedef ValidType<unsigned long> vulong;


   // Yes, Virgina, this is the ugliest declaration that I have ever created...
   template <class T> std::ostream& operator<<(
      std::ostream& s, const ValidType<T>& r) throw() {
      if (r.is_valid())
         s << r.get_value();
      else
         s << "Unknown";
      return s;
   }

}

#endif
