#pragma ident "$Id$"

/**
 * @file BaseDistribution.hpp
 * This is a base class for statistical distributions.
 */

#ifndef BASEDISTRIBUTION_HPP
#define BASEDISTRIBUTION_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008
//
//============================================================================


#include <cmath>
#include "SpecialFunctions.hpp"
#include "Exception.hpp"


namespace gpstk
{

      /** @addtogroup math */
      //@{

      /// This is a base class for statistical distributions.
   class BaseDistribution
   {
   public:


         /// Computes the probability density function
      virtual double pdf(double x) = 0;


         /// Computes the cumulative distribution function
      virtual double cdf(double x) = 0;


   };  // End of class "BaseDistribution"

      //@}

}  // End of namespace gpstk
#endif   // BASEDISTRIBUTION_HPP
