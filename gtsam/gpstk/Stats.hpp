#pragma ident "$Id$"

/**
 * @file Stats.hpp
 * One and two-sample statistics
 */
 
#ifndef INCLUDE_GPSTK_STATS_HPP
#define INCLUDE_GPSTK_STATS_HPP

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

namespace gpstk
{
   /** @addtogroup math */
   //@{
 
   /// Conventional statistics for one sample.  Constructor does the same as
   /// Reset(); use this when starting a new series of input samples.
   /// Results are available at any time by calling N(), Minimum(), Maximum(),
   /// Average(), Variance() and StdDev().
   /// NB. Variance is normalized with 1/(N-1) and StdDev is sqrt(Variance).
   /// NB. This class may not give exactly correct results with non-floating types,
   ///     (for which it is not intended).
   template <class T>
   class Stats
   {
   public:
      /// constructor
      explicit Stats() { n=0; weighted=false; }

      /// constructor given a vector of data
      Stats(Vector<T>& X, Vector<T>& W)
      {
         n = 0;
         weighted = false;
         Add(X,W);
      }

      /// reset, i.e. ignore earlier data and restart sampling
      inline void Reset(void) { n=0; weighted=false; W=T(); }

      /// access the sample size
      inline unsigned int N(void) const { return n; }

      /// return minimum value
      inline T Minimum(void) const { if(n) return min; else return T(); }

      /// return maximum value
      inline T Maximum(void) const { if(n) return max; else return T(); }

      /// return computed average
      inline T Average(void) const
      {
         if(n == 0) return T();
         return ave;
      }

      /// return computed variance
      inline T Variance(void) const
      {
         if(n <= 1) return T();
         return (T(n)*var/T(n-1));
      }

      /// return computed standard deviation
      inline T StdDev(void) const
      {
         if(n <= 1) return T();
         return SQRT(Variance());
      }

      /// return the normalization constant = sum weights
      inline T Normalization(void) const { return W; }

      /// return weight flag
      inline bool Weighted(void) const { return weighted; }

      /// add a single sample to the computation of statistics, with optional weight
      void Add(const T& x, const T& wt_in=T())
      {
         T wt(ABS(wt_in));
         if(wt != T()) { weighted=true; }

         if(n == 0) {
            min = max = ave = x;
            var = T();
            W = T();
         }
         else {
            if(x < min) min=x;
            if(x > max) max=x;
         }

         if(weighted) {
            if(W+wt > T(1.e-10))     // if W+wt=0, nothing yet has had non-zero weight
               ave += (x-ave)*(wt/(W+wt));
            if(n > 0 && W > 1.e-10)
               var = (W/(W+wt))*var + (x-ave)*(x-ave)*(wt/W);
            W += wt;
         }
         else {
            ave += (x-ave)/T(n+1);
            if(n > 0)
               var = T(n)*var/T(n+1) + (x-ave)*(x-ave)/T(n);
         }

         n++;
      }

      /// add a Vector<T> of samples to the computation of statistics,
      /// with optional weights
      inline void Add(Vector<T>& X, Vector<T> w = Vector<T>())
      {
         if(w.size() > 0 && w.size() < X.size()) {
            Exception e("Inconsistent input: weight vector short");
            GPSTK_THROW(e);
         }

         size_t i;
         if(w.size() > 0) 
            for(i=0; i<X.size(); i++) Add(X(i),w(i));
         else
            for(i=0; i<X.size(); i++) Add(X(i));
      }

      /// remove a sample from the computation of statistics (can't do min and max).
      /// TD test this...
      void Subtract(const T x, const T wt_in=T())
      {
         if(n == 0) return;
         if(weighted) {
            if(W > T(1.e-10)) {
               T wt(ABS(wt_in));
               if(W-wt > T(1.e-10))
                  var = (var - (wt/(W-wt))*(x-ave)*(x-ave)) * (W/(W-wt));
               else
                  var = T();
               ave = (ave - wt*x/W)*W/(W-wt);
               W -= wt;
            }
            else { ave = var = W = T(); }
         }
         else {
            if(n > 1)
               var = (var - (x-ave)*(x-ave)/T(n-1))*T(n)/(T(n)-T(1));
            else
               var = T();
            ave = T(n)*(ave - x/T(n))/(T(n)-T(1));
         }
         n--;
      }

      /// remove a Vector<T> of samples to the computation of statistics
      inline void Subtract(Vector<T>& X)
      {
         for(size_t i=0; i<X.size(); i++) Subtract(X(i));
      }

      /// define private members directly; useful in continuing with an object
      /// that was earlier saved (e.g. to a file).
      void Load(unsigned int in_n, T in_min, T in_max, T in_ave, T in_var,
                  bool wtd=false, T norm=1.0)
      {
         n = in_n;
         min = in_min;
         max = in_max;
         var = in_var;
         ave = in_ave;
         weighted = wtd;
         W = norm;
      }
      
      /// combine two Stats (assumed taken from the same or equivalent samples);
      /// both must be either weighted or unweighted.
      /// NB. Beware using this with very small sample size.
      Stats<T>& operator+=(const Stats<T>& S)
      {
         if(S.n == 0) return *this;
         if(n==0) { *this = S; return *this; }
         if((weighted && !S.weighted) || (!weighted && S.weighted)) {
            Exception e("Stats::operator+= : operands have inconsistent weighting");
            GPSTK_THROW(e);
         }

         if(S.min < min) min=S.min;
         if(S.max > max) max=S.max;

         T newave, newvar;
         if(weighted) {
            if(W + S.W > T(1.e-10)) {
               newave = W*ave + S.W*S.ave;
               newvar = W*var + S.W*S.var + W*ave*ave + S.W*S.ave*S.ave;
               W += S.W;
               ave = newave/W;
               //var = (newvar-W*ave*ave)/W;
               var = newvar/W -ave*ave;
            }
         }
         else {
            newave = T(n)*ave + T(S.n)*S.ave;
            newvar = T(n)*var + T(S.n)*S.var + T(n)*ave*ave + T(S.n)*S.ave*S.ave;
            ave = newave/T(n+S.n);
            //var = (newvar-T(n+S.n)*ave*ave)/T(n+S.n);
            var = newvar/T(n+S.n) - ave*ave;
         }
         n += S.n;

         return *this;

      }  // end Stats operator+=

      /// dump the data stored in the class

   private:
      /// Number of samples added to the statistics so far
      unsigned int n;

      /// Minimum value
      T min;

      /// Maximum value
      T max;

      /// Average value
      T ave;

      /// Variance (square of the standard deviation)
      T var;

      /// Normalization constant = sum weights
      T W;

      /// Flag weighted input; ALL input must be consistently weighted or not weighted
      bool weighted;

   }; // end class Stats

   /// Output operator for Stats class
   template <class T>
   std::ostream& operator<<(std::ostream& s, const Stats<T>& ST) 
   {
      std::ofstream savefmt;
      savefmt.copyfmt(s);
      s << " N       = " << ST.N() << (ST.Weighted() ? " ":" not") << " weighted\n";
      s << " Minimum = "; s.copyfmt(savefmt); s << ST.Minimum();
      s << " Maximum = "; s.copyfmt(savefmt); s << ST.Maximum() << "\n";
      s << " Average = "; s.copyfmt(savefmt); s << ST.Average();
      s << " Std Dev = "; s.copyfmt(savefmt); s << ST.StdDev();
      s << " Variance = "; s.copyfmt(savefmt); s << ST.Variance(); // temp
      return s;
   }

   /// Conventional statistics for two samples.  Constructor does the same as
   /// Reset(); use this when starting a new series of input samples.
   /// Results are available at any time by calling N(), Minimum(), Maximum(),
   /// Average(), Variance() and StdDev().
   /// NB. Variance is normalized with 1/(N-1) and StdDev is sqrt(Variance).
   /// NB. This class may not give exactly correct results with non-floating types,
   ///     (for which it is not intended).
   template <class T>
   class TwoSampleStats
   {
   public:
      /// constructor
      TwoSampleStats() { n=0; }

      /// constructor given two Vector of data
      TwoSampleStats(Vector<T>& X, Vector<T>& Y)
      {
         n = 0;
         Add(X,Y);
      }

      /// Add data to the statistics
      void Add(const T& X, const T& Y)
      {
         if(n == 0) {
            sumx = sumy = sumx2 = sumy2 = sumxy = T();
            xmin = xmax = X;
            ymin = ymax = Y;
            scalex = scaley = T(1);
         }
         if(scalex==T(1) && X!=T()) scalex=ABS(X);
         if(scaley==T(1) && Y!=T()) scaley=ABS(Y);
         sumx += X/scalex;
         sumy += Y/scaley;
         sumx2 += (X/scalex)*(X/scalex);
         sumy2 += (Y/scaley)*(Y/scaley);
         sumxy += (X/scalex)*(Y/scaley);
         if(X < xmin) xmin=X;
         if(X > xmax) xmax=X;
         if(Y < ymin) ymin=Y;
         if(Y > ymax) ymax=Y;
         n++;
      }

      /// Add two Vectors of data to the statistics
      void Add(const Vector<T>& X, const Vector<T>& Y)
      {
         size_t m = (X.size() < Y.size() ? X.size() : Y.size());
         if(m==0) return;
         for(size_t i=0; i<m; i++) Add(X(i),Y(i));
      }

      void Subtract(const T& X, const T& Y)
      {
         if(n == 1) {
            sumx = sumy = sumx2 = sumy2 = sumxy = T();
            xmin = xmax = T();
            ymin = ymax = T();
            scalex = scaley = T(1);
            return;
         }

         sumx -= X/scalex;
         sumy -= Y/scaley;
         sumx2 -= (X/scalex)*(X/scalex);
         sumy2 -= (Y/scaley)*(Y/scaley);
         sumxy -= (X/scalex)*(Y/scaley);
         n--;
      }

      void Subtract(const Vector<T>& X, const Vector<T>& Y)
      {
         size_t m=(X.size()<Y.size()?X.size():Y.size());
         if(m==0) return;
         for(size_t i=0; i<m; i++) Subtract(X(i),Y(i));
      }

      /// reset, i.e. ignore earlier data and restart sampling
      inline void Reset(void) { n=0; }

      /// return the sample size
      inline unsigned int N(void) const { return n; }
      /// return minimum X value
      inline T MinimumX(void) const { if(n) return xmin; else return T(); }
      /// return maximum X value
      inline T MaximumX(void) const { if(n) return xmax; else return T(); }
      /// return minimum Y value
      inline T MinimumY(void) const { if(n) return ymin; else return T(); }
      /// return maximum Y value
      inline T MaximumY(void) const { if(n) return ymax; else return T(); }

      /// return computed X average
      inline T AverageX(void) const
         { if(n>0) return (scalex*sumx/T(n)); else return T(); }

      /// return computed Y average
      inline T AverageY(void) const
         { if(n>0) return (scaley*sumy/T(n)); else return T(); }

      /// return computed X variance
      inline T VarianceX(void) const
      {
         if(n>1) return scalex*scalex*(sumx2-sumx*sumx/T(n))/T(n-1);
         else return T();
      }

      /// return computed Y variance
      inline T VarianceY(void) const
      {
         if(n>1) return scaley*scaley*(sumy2-sumy*sumy/T(n))/T(n-1);
         else return T();
      }

      /// return computed X standard deviation
      inline T StdDevX(void) const { return SQRT(VarianceX()); }

      /// return computed Y standard deviation
      inline T StdDevY(void) const { return SQRT(VarianceY()); }

      /// return slope of best-fit line Y=slope*X + intercept
      inline T Slope(void) const
      {
         if(n>0)
            return ((scaley/scalex)*(sumxy-sumx*sumy/T(n))/(sumx2-sumx*sumx/T(n)));
         else
            return T();
      }

      /// return intercept of best-fit line Y=slope*X + intercept
      inline T Intercept(void) const
      {
         if(n>0)
            return (AverageY()-Slope()*AverageX());
         else
            return T();
      }

      /// return uncertainty in slope
      inline T SigmaSlope(void) const
      {
         if(n>2)
            return (SigmaYX()/(StdDevX()*SQRT(T(n-1))));
         else
            return T();
      }

      /// return correlation
      inline T Correlation(void) const
      {
         if(n>1)
         {
            return ( scalex * scaley * (sumxy-sumx*sumy/T(n))
               / (StdDevX()*StdDevY()*T(n-1)) );
         }
         else
            return T();
      }

      /// return conditional uncertainty = uncertainty y given x
      inline T SigmaYX(void) const
      {
         if(n>2)
         {
            return (StdDevY() * SQRT(T(n-1)/T(n-2))
                  * SQRT(T(1)-Correlation()*Correlation()) );
         }
         else return T();
      }

      /// combine two TwoSampleStats (assumed to be taken from the same or
      /// equivalent samples)
      /// NB. Beware using this with very small sample size.
      TwoSampleStats<T>& operator+=(TwoSampleStats<T>& S)
      {
         if(n + S.n == 0) return *this;
         if(S.xmin < xmin) xmin=S.xmin;
         if(S.xmax > xmax) xmax=S.xmax;
         if(S.ymin < ymin) ymin=S.ymin;
         if(S.ymax > ymax) ymax=S.ymax;
         sumx += S.scalex*S.sumx/scalex;
         sumy += S.scaley*S.sumy/scaley;
         sumx2 += (S.scalex/scalex)*(S.scalex/scalex)*S.sumx2;
         sumy2 += (S.scaley/scaley)*(S.scaley/scaley)*S.sumy2;
         sumxy += (S.scalex/scalex)*(S.scaley/scaley)*S.sumxy;
         n += S.n;
         return *this;
      }  // end TwoSampleStats operator+=

   private:
      /// Number of samples added to the statistics so far
      unsigned int n;
      T xmin, xmax, ymin, ymax, scalex, scaley;
      T sumx, sumy, sumx2, sumy2, sumxy;

   }; // end class TwoSampleStats

   /// Output operator for TwoSampleStats class
   template <class T>
   std::ostream& operator<<(std::ostream& s, const TwoSampleStats<T>& TSS) 
   {
      std::ofstream savefmt;
      savefmt.copyfmt(s);
      s << " N           = " << TSS.N() << "\n";
      s << " Minimum:  X = "; s.copyfmt(savefmt); s << TSS.MinimumX();
      s << "  Y = "; s.copyfmt(savefmt); s << TSS.MinimumY() << "\n";
      s << " Maximum:  X = "; s.copyfmt(savefmt); s << TSS.MaximumX();
      s << "  Y = "; s.copyfmt(savefmt); s << TSS.MaximumY() << "\n";
      s << " Average:  X = "; s.copyfmt(savefmt); s << TSS.AverageX();
      s << "  Y = "; s.copyfmt(savefmt); s << TSS.AverageY() << "\n";
      s << " Std Dev:  X = "; s.copyfmt(savefmt); s << TSS.StdDevX();
      s << "  Y = "; s.copyfmt(savefmt); s << TSS.StdDevY() << "\n";
      s << " Variance: X = "; s.copyfmt(savefmt); s << TSS.VarianceX();
      s << "  Y = "; s.copyfmt(savefmt); s << TSS.VarianceY() << "\n";
      s << " Intercept = "; s.copyfmt(savefmt); s << TSS.Intercept();
      s << "  Slope = "; s.copyfmt(savefmt); s << TSS.Slope();
      s << " with uncertainty = "; s.copyfmt(savefmt); s << TSS.SigmaSlope() << "\n";
      s << " Conditional uncertainty (sigma y given x) = ";
      s.copyfmt(savefmt); s << TSS.SigmaYX();
      s << "  Correlation = "; s.copyfmt(savefmt); s << TSS.Correlation();
      return s;
   }

   /// Compute the median of a gpstk::Vector
   template <class T>
   inline T median(const Vector<T>& v)
   {
      if(v.size()==0) return T();
      if(v.size()==1) return v(0);
      if(v.size()==2) return (v(0)+v(1))/T(2);
      // insert sort
      int i,j;
      T x;
      Vector<T> w(v);
      for(i=0; i<v.size(); i++) {
         x = w[i] = v(i);
         j = i-1;
         while(j>=0 && x<w[j]) {
            w[j+1] = w[j];
            j--;
         }
         w[j+1] = x;
      }
      if(v.size() % 2)
         x=w[(v.size()+1)/2-1];
      else
         x=(w[v.size()/2-1]+w[v.size()/2])/T(2);

      return x;
   }  // end median(Vector)

   /// Compute the median of a std::vector
   template <class T>
   inline T median(const std::vector<T>& v)
   {
      if(v.size()==0) return T();
      if(v.size()==1) return v[0];
      if(v.size()==2) return (v[0]+v[1])/T(2);
      // insert sort
      int j;
      size_t i;
      T x;
      std::vector<T> w(v);
      for(i=0; i<v.size(); i++) {
         x = w[i] = v[i];
         j = i-1;
         while(j>=0 && x<w[j]) {
            w[j+1] = w[j];
            j--;
         }
         w[j+1] = x;
      }
      if(v.size() % 2)
         x=w[(v.size()+1)/2-1];
      else
         x=(w[v.size()/2-1]+w[v.size()/2])/T(2);

      return x;
   }  // end median(Vector)

   //@}

}  // namespace

#endif
