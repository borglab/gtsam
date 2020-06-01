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

#include "PowerSum.hpp"
#include <math.h>

namespace gpstk
{
   void PowerSum::clear() throw()
   {
      for (int i=1; i<=order; i++)
         s[i]=0.0;
      n=0;
   }
   
   void PowerSum::add(double x) throw()
   {
      n++;
      double px=x;
      for (int i=1; i<=order; i++, px*=x)
         s[i] += px;
   }

   void PowerSum::subtract(double x) throw()
   {
      n--;
      double px=x;
      for (int i=1; i<=order; i++, px*=x)
         s[i] -= px;
   }

   void PowerSum::add(dlc_iterator b, dlc_iterator e) throw()
   {
      dlc_iterator i;
      for (i=b; i != e; i++)
         add(*i);
   }

   void PowerSum::subtract(dlc_iterator b, dlc_iterator e) throw()
   {
      dlc_iterator i;
      for (i=b; i != e; i++)
         subtract(*i);
   }

   /// See http://mathworld.wolfram.com/SampleCentralMoment.html for
   /// computing the central moments from the power sums.
   double PowerSum::moment(int i) const throw()
   {
      if ( i > order || i >= n)
         return 0;

      double m=0;
      double ni=1.0/n;
      double s12 = s[1]*s[1];

      if (i==1 && n>0)
         m = ni*s[1];
      if (i==2 && n>1)
         m = ni*(s[2] - ni*s12);
      else if (i==3 && n>2)
         m = ni*(s[3] + ni*(-3*s[1]*s[2] + ni*(2*s12*s[1])));
      else if (i==4 && n>3)
         m = ni*(s[4] + ni*(-4*s[1]*s[3] + ni*(6*s12*s[2] + ni*(-3*s12*s12))));
      else if (i==5 && n>4)
         m = ni*(s[5] + ni*(-5*s[1]*s[4] +ni*(10*s12*s[3]
             + ni*(-10*s12*s[1]*s[2] + ni*(4*s12*s12*s[1])))));

      return m;
   }

   double PowerSum::average() const throw()
   {
      if (n<1)
         return 0;
      return s[1]/n;
   }


   double PowerSum::variance() const throw()
   {
      if (n<2)
         return 0;
      return moment(2);
   }

   double PowerSum::skew() const throw()
   {
      if (n<3)
         return 0;
      return moment(3)/pow(moment(2),1.5);
   }

   double PowerSum::kurtosis() const throw()
   {
      if (n<4)
         return 0;
      double m2 = moment(2);
      return moment(4)/(m2*m2);
   }

   void PowerSum::dump(std::ostream& str) const throw()
   {
      str << "n:" << n;
      for (int i=1; i<=order; i++)
         str << " s" << i << ":" << s[i];
      str << std::endl;

      str << "m1:" << moment(1)
          << " m2:" << moment(2)
          << " m3:" << moment(3)
          << " m4:" << moment(4)
          << std::endl;

      str << "average:" << average()
          << " stddev:" << sqrt(variance()) 
          << " skew:" << skew()
          << " kurtosis:" << kurtosis()
          << std::endl;
   }   
}
