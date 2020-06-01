#pragma ident "$Id$"



/**
 * @file BivarStats.hpp
 * Bivariate Statistics
 */
 
#ifndef INCLUDE_GPSTK_BIVARSTATS_HPP
#define INCLUDE_GPSTK_BIVARSTATS_HPP

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

#include "MiscMath.hpp"
#include "Vector.hpp"
#include "Exception.hpp"
#include "Stats.hpp"

namespace gpstk
{

      /** @addtogroup math */
      /** @{ */
 
      /** Conventional statistics for two samples.  Constructor does the same as
       * clear(); use this when starting a new series of input samples.
       */
   template <class T>
   class BivarStats
   {
   public:
         /** @name Constructors.
          * Various ways to construct a BivarStats object.
          * @param x the data to use for the independent variable
          * @param y the data to use for the dependent variable
          * @param scale whether data is scaled internally (default: false) */
         /** @{ */
      BivarStats(bool scale=false);
      BivarStats(const T& x, const T&, bool scale=false);
      BivarStats(const std::vector<T>& x, const std::vector<T>& y, 
                 bool scale=false);
      BivarStats(const std::vector< std::pair<T, T> >& d, bool scale=false);
      BivarStats(const Vector<T>& x, const Vector<T>& y, bool scale=false);
         /** @} */

         /** @name Addition Functions
          * Add data to the statistics. */
         /** @{ */
      void add(const T& x, const T& y);
      void add(const std::vector<T>& x, const std::vector<T>& y);
      void add(const std::vector< std::pair<T, T> >& d);
      void add(const Vector<T>& x, const Vector<T>& y);
         /** @} */
      
         /** @name Subtraction Functions
          * Subtract data from the statistics. */
         /** @{ */
      void subtract(const T& x, const T& y);
      void subtract(const std::vector<T>& x, const std::vector<T>& y);
      void subtract(const std::vector< std::pair<T, T> >& d);
      void subtract(const Vector<T>& x, const Vector<T>& y);
         /** @} */

      void clear(void); ///< Remove all data and start over.
      size_t n(void) const; ///< Return the sample size.

      T minimumX(void) const;
      T maximumX(void) const;
      T minimumY(void) const;
      T maximumY(void) const;

      T averageX(void) const;
      T averageY(void) const;

      T varianceX(void) const;
      T varianceY(void) const;
      T stdDevX(void) const;
      T stdDevY(void) const;

         /// Return slope of best-fit line Y=slope*X + intercept.
      T slope(void) const;
         /// Return intercept of best-fit line Y=slope*X + intercept
      T intercept(void) const; 
         /// Return uncertainty in slope.
      T sigmaSlope(void) const;

      T correlation(void) const;

      /// return conditional uncertainty = uncertainty y given x
      T sigmaYX(void) const;

      /// compute intercept + x * slope
      T eval(const T& x) const {return intercept() + x * slope();};

      /// Combine two BivarStats (assumed to be taken from the same or
      /// equivalent samples).
      BivarStats<T>& operator+=(BivarStats<T>& S);

      Stats<T> estimateDeviation(const std::vector< std::pair<T, T> >& d) const;

   private:

      /// Number of samples added to the statistics so far.
      size_t ns;

      T xMin, xMax, yMin, yMax;
      T scaleX, scaleY;
      bool scaled;
      T sumX, sumY, sumX2, sumY2, sumXY;
   }; // end class BivarStats

   /// Output operator for BivarStats class
   template <class T>
   std::ostream& operator<<(std::ostream& s, const BivarStats<T>& BVS) 
   {
      s << " N       = " << BVS.n() << std::endl
        << " Minimum: X = " << BVS.minimumX()
        << "  Y = " << BVS.minimumY()
        << "  Maximum: X = " << BVS.maximumX()
        << "  Y = " << BVS.maximumY() << std::endl
        << " Average: X = " << BVS.averageX()
        << "  Y = " << BVS.averageY()
        << "  Std Dev: X = " << BVS.stdDevX()
        << "  Y = " << BVS.stdDevY() << std::endl
        << " Intercept = " << BVS.intercept()
        << "  Slope = " << BVS.slope()
        << " with uncertainty = " << BVS.sigmaSlope() << std::endl
        << " Conditional uncertainty (sigma y given x) = " << BVS.sigmaYX()
        << "  Correlation = "  << BVS.correlation() << std::endl;
      return s;
   }

   template<class T>
   BivarStats<T>::BivarStats(bool s)
         :ns(0), scaled(s)
   {}

   template<class T>
   BivarStats<T>::BivarStats(const T& x, const T& y, bool s)
         :ns(0), scaled(s)
   {
      add(x,y);
   }
   
   template<class T>
   BivarStats<T>::BivarStats(const std::vector<T>& x, const std::vector<T>& y, 
                             bool s )
         :ns(0), scaled(s)
   {
      add(x,y);
   }

   template<class T>
   BivarStats<T>::BivarStats(const std::vector< std::pair<T, T> >& d, bool s)
         :ns(0), scaled(s)
   {
      add(d);
   }

   template<class T>
   BivarStats<T>::BivarStats(const Vector<T>& x, const Vector<T>& y, bool s)
      :ns(0), scaled(s)
   {
      add(x,y);
   }

   template<class T>
   void BivarStats<T>::add(const T& x, const T& y)
   {
      if (ns == 0)
      {
         sumX = sumY = sumX2 = sumY2 = sumXY = T(0);
         xMin = xMax = x;
         yMin = yMax = y;
         scaleX = scaleY = T(1);
      }

      if (scaled)
      {
         if (scaleX==T(1) && x!=T()) scaleX=ABS(x);
         if (scaleY==T(1) && y!=T()) scaleY=ABS(y);
         T tx(x/scaleX);
         T ty(y/scaleY);
         sumX += tx;
         sumY += ty;
         sumX2 += tx*tx;
         sumY2 += ty*ty;
         sumXY += tx*ty;
      }
      else
      {
         sumX += x;
         sumY += y;
         sumX2 += x*x;
         sumY2 += y*y;
         sumXY += x*y;
      }

      if(x < xMin) xMin=x;
      if(x > xMax) xMax=x;
      if(y < yMin) yMin=y;
      if(y > yMax) yMax=y;
      ns++;
   }

   template<class T>
   void BivarStats<T>::add(const std::vector<T>& x, const std::vector<T>& y)
   {
      size_t m = x.size() < y.size() ? x.size() : y.size();
      if(m==0)
         return;
      for (size_t i=0; i<m; i++)
         add(x[i], y[i]);
   }

   template<class T>
   void BivarStats<T>::add(const std::vector< std::pair<T, T> >& d)
   {
      size_t max( d.size() );
      for (size_t i=0; i<max; i++)
         add(d[i].first, d[i].second);
   }

   template<class T>
   void BivarStats<T>::add(const Vector<T>& x, const Vector<T>& y)
   {
      size_t m = x.size() < y.size() ? x.size() : y.size();
      if (m==0)
         return;
      for (size_t i=0; i<m; i++)
         add(x(i), y(i));
   }

   template<class T>
   void BivarStats<T>::subtract(const T& x, const T& y)
   {
      if (ns < 2)
      {
         ns = 0;
         return;
      }

      if (scaled)
      {
         T tx(x/scaleX);
         T ty(y/scaleY);
         sumX -= tx;
         sumY -= ty;
         sumX2 -= tx*tx;
         sumY2 -= ty*ty;
         sumXY -= tx*ty;
      }
      else
      {
         sumX -= x;
         sumY -= y;
         sumX2 -= x*x;
         sumY2 -= y*y;
         sumXY -= x*y;
      }

      ns--;
   }

   template<class T>
   void BivarStats<T>::subtract(const std::vector<T>& x, const std::vector<T>& y)
   {
      size_t m = x.size()<y.size() ? x.size() : y.size();
      if(m==0)
         return;
      for (size_t i=0; i<m; i++)
         subtract(x[i], y[i]);
   }

   template<class T>
   void BivarStats<T>::subtract(const std::vector< std::pair<T, T> >& d)
   {
      size_t max( d.size() );
      for (size_t i=0; i<max; d++)
         subtract(d[i].first, d[i].second);
   }

   template<class T>
   void BivarStats<T>::subtract(const Vector<T>& x, const Vector<T>& y)
   {
      size_t m = x.size()<y.size() ? x.size() : y.size();
      if (m==0)
         return;
      for (size_t i=0; i<m; i++)
         subtract(x(i), y(i));
   }

   /// This assumes that the accessors will check for n>0, which they do.
   template<class T>
   void BivarStats<T>::clear(void) { ns=0; }

   template<class T>
   inline size_t BivarStats<T>::n(void) const { return ns; }

   template<class T>
   T BivarStats<T>::minimumX(void) const { return ns>0 ? xMin : T(0); }
   template<class T>
   T BivarStats<T>::maximumX(void) const { return ns>0 ? xMax : T(0); }
   template<class T>
   T BivarStats<T>::minimumY(void) const { return ns>0 ? yMin : T(0); }
   template<class T>
   T BivarStats<T>::maximumY(void) const { return ns>0 ? yMax : T(0); }

   template<class T>
   T BivarStats<T>::averageX(void) const
   { return ns>0 ? scaleX*sumX/T(ns) : T(0); }
   template<class T>
   T BivarStats<T>::averageY(void) const 
   { return ns>0 ? scaleY*sumY/T(ns) : T(0); }

   
   template<class T>
   T BivarStats<T>::varianceX(void) const
   {  
      return (ns>1) ? scaleX*scaleX * (sumX2 - sumX*sumX/T(ns)) / T(ns-1) : T(0); 
   }

   template<class T>
   T BivarStats<T>::varianceY(void) const
   {
      return (ns>1) ? scaleY*scaleY * (sumY2 - sumY*sumY/T(ns)) / T(ns-1) : T(0);
   }

   template<class T>
   T BivarStats<T>::stdDevX(void) const { return SQRT(varianceX()); }
   template<class T>
   T BivarStats<T>::stdDevY(void) const { return SQRT(varianceY()); }

   template<class T>
   T BivarStats<T>::slope(void) const
   {
      if (ns>0)
         return (scaleY/scaleX) * (sumXY - sumX*sumY/T(ns)) / 
            (sumX2 - sumX*sumX/T(ns));
      else
         return T();
   }

   template<class T>
   T BivarStats<T>::intercept(void) const
   {
      if (ns>0)
         return averageY() - slope() * averageX();
      else
         return T();
   }
   
   template<class T>
   T BivarStats<T>::sigmaSlope(void) const
   {
      if (ns>2)
         return sigmaYX() / (stdDevX() * SQRT(T(ns-1)));
      else
         return T();
   }
   
   template<class T>
   T BivarStats<T>::correlation(void) const
   {
      if (ns>1)
         return scaleX*scaleY * (sumXY - sumX*sumY/T(ns)) /
            (stdDevX() * stdDevY() * T(ns-1));
      else
         return T();
   }

   template<class T>
   T BivarStats<T>::sigmaYX(void) const
   {
      if (ns>2)
         return (stdDevY() * SQRT(T(ns-1) / T(ns-2))
                 * SQRT(T(1) - correlation() * correlation()) );
      else return T();
   }

   /// combine two BivarStats (assumed to be taken from the same or
   /// equivalent samples)
   template<class T>
   BivarStats<T>& BivarStats<T>::operator+=(BivarStats<T>& S)
   {
      if(ns + S.ns == 0) return *this;
      xMin = std::min(xMin, S.xMin);
      xMax = std::max(xMax, S.xMax);
      yMin = std::min(yMin, S.yMin);
      yMax = std::max(yMax, S.yMax);
      T xscaler( S.scaleX/scaleX ), yscaler( S.scaleY/scaleY ); 
      sumX += xscaler * S.sumX;
      sumY += yscaler * S.sumY;
      sumX2 += xscaler * xscaler * S.sumX2;
      sumY2 += yscaler * yscaler * S.sumY2;
      sumXY += xscaler * yscaler * S.sumXY;
      ns += S.ns;
      return *this;
   }
      /** @} */ // end of  @addtogroup math

   template<class T>
   Stats<T> BivarStats<T>::estimateDeviation(const std::vector< std::pair<T, T> >& d) const
   {
      Stats<T> estats;
      size_t max( d.size() );
      for (size_t i=0; i<max; i++)
         estats.Add(std::abs(d[i].second - eval(d[i].first)));
      return estats;
   }

}  // namespace

#endif
