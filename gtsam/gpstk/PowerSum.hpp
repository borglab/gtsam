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
#ifndef GPSTK_POWERSUM_HPP
#define GPSTK_POWERSUM_HPP

#include <iostream>
#include <list>

namespace gpstk
{
   /* This class computes the power sums of a list of numbers and computes
      various statistical values based upon these sums. This is a generalization
      of the Stats class that supports the higher-order moments. See
      http://mathworld.wolfram.com/PowerSum.html for a discussion of this approach.
   */
   class PowerSum
   {
   public:
      PowerSum() {clear();};

      const static int order = 5;

      /// Reset all sums
      void clear() throw();

      /// Add a single value to the sums
      void add(double x) throw();

      /// Remove a single value from the sums. Note that the higher order sums
      /// can get quite large in magnitude. When removing a value that is
      /// far from the average, it is possible for numerical error to creep
      /// into the sums. One way around this is to simply recompute the sums
      /// from scratch when this happens.
      void subtract(double x) throw();

      typedef std::list<double>::const_iterator dlc_iterator;

      /// Adds all value in the list to the sums.
      void add(dlc_iterator b, dlc_iterator e) throw();

      /// Removes all values in the list from the sums. See the warning with the
      /// subtract(double) method.
      void subtract(dlc_iterator b, dlc_iterator e) throw();

      /// Computes the ith order central moment
      double moment(int i) const throw();

      /// Reuturn the number of points in the current sum
      long size() const throw() {return n;}
      
      /// Computes the indicated value
      double average() const throw();
      double variance() const throw();
      double skew() const throw();
      double kurtosis() const throw();

      void dump(std::ostream& str) const throw();

   private:
      double s[order+1];
      long n;
/*
  These are used to determine kurtosis values for specific confidence based upon
  the sample size.
  const double pnt[]={
  5,    7,    8,    9,   10,   12,   15,   20,   25,
  30,   40,   50,   75,  100,  200,  500, 1000, 1e5, 1.e30};
  // 5% critical values
  const double cv5[] = {
  2.90, 3.55, 3.70, 3.86, 3.95, 4.05, 4.13, 4.17, 4.16,
  4.11, 4.06, 3.99, 3.87, 3.77, 3.57, 3.37, 3.26, 3.10, 3.00};
  // 1% critical values
  const double cv1[]= {
  3.10, 4.23, 4.53, 4.82, 5.00, 5.20, 5.30, 5.36, 5.30,
  5.21, 5.04, 4.88, 4.59, 4.39, 3.98, 3.60, 3.41, 3.20, 3.00};
*/
   };

}
#endif
