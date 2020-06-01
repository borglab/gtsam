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
 * @file XvtStore.hpp
 * Abstract base class for storing and/or computing position, velocity, 
 * and clock data.
 */

#ifndef GPSTK_XVTSTORE_INCLUDE
#define GPSTK_XVTSTORE_INCLUDE

#include <iostream>

#include "Exception.hpp"
#include "CommonTime.hpp"
#include "Xvt.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Abstract base class for storing and accessing an object's position, 
   /// velocity, and clock data. Also defines a simple interface to remove
   /// data that had been added.
   template <class IndexType>
   class XvtStore
   {
   public:
      virtual ~XvtStore()
      {}

      /// Returns the position, velocity, and clock offset of the indicated
      /// object in ECEF coordinates (meters) at the indicated time.
      /// @param[in] id the object's identifier
      /// @param[in] t the time to look up
      /// @return the Xvt of the object at the indicated time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The text may have additional
      ///    information as to why the request failed.
      virtual Xvt getXvt(const IndexType& id, const CommonTime& t) const = 0;

      /// A debugging function that outputs in human readable form,
      /// all data stored in this object.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      virtual void dump(std::ostream& s = std::cout, short detail = 0) const = 0;

      /// Edit the dataset, removing data outside the indicated time interval
      /// @param[in] tmin defines the beginning of the time interval
      /// @param[in] tmax defines the end of the time interval
      virtual void edit(const CommonTime& tmin, 
                        const CommonTime& tmax = CommonTime::END_OF_TIME) = 0;

      /// Clear the dataset, meaning remove all data
      virtual void clear(void) = 0;

      /// Return the time system of the store
      virtual TimeSystem getTimeSystem(void) const = 0;

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return The initial time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getInitialTime(void) const = 0;

      /// Determine the latest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return The final time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getFinalTime(void) const = 0;

      /// Return true if velocity data is present in the store
      virtual bool hasVelocity(void) const = 0;

      /// Return true if the given IndexType is present in the store
      virtual bool isPresent(const IndexType& id) const = 0;

   }; // end class XvtStore

   //@}

} // namespace

#endif // GPSTK_XVTSTORE_INCLUDE
