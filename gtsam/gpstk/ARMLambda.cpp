#pragma ident "$Id$"

/**
 * @file ARMLambda.cpp
 * 
 */
 
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
//  Wei Yan - Chinese Academy of Sciences . 2011
//
//============================================================================

#include "ARMLambda.hpp"


namespace gpstk
{

   int ARMLambda::search( const Matrix<double>& L, 
                          const Vector<double>& D, 
                          const Vector<double>& zs, 
                          Matrix<double>& zn, 
                          Vector<double>& s, 
                          const int& m )
   {
      // TODO: CHECK UNEXPECTED INPUT
      // n - number of float parameters
      // m - number of fixed solutions
      // L - nxn
      // D - nx1
      // zs - nxn
      // zn - nxm
      // s  - m
      const int LOOPMAX = 10000;
      const int n = L.rows();

      zn.resize(n,m,0.0);
      s.resize(m,0.0);

      Matrix<double> S(n,n,0.0);
      Vector<double> dist(n,0.0),zb(n,0.0),z(n,0.0),step(n,0.0);

      int k=n-1; dist[k]=0.0;
      zb(k)=zs(k);
      z(k)=round(zb(k)); 
      double y=zb(k)-z(k); 
      step(k)=sign(y);

      int c(0),nn(0),imax(0);
      double maxdist=1E99;
      for(int c=0;c<LOOPMAX;c++)
      {
         double newdist=dist(k)+y*y/D(k);
         if(newdist<maxdist) 
         {
            if(k!=0) 
            {
               dist(--k)=newdist;
               for(int i=0;i<=k;i++)
               {
                  S(k,i)=S(k+1,i)+(z(k+1)-zb(k+1))*L(k+1,i);
               }
               zb(k)=zs(k)+S(k,k);
               z(k)=round(zb(k)); y=zb(k)-z(k); step(k)=sign(y);
            }
            else 
            {
               if(nn<m) 
               {
                  if(nn==0||newdist>s(imax)) imax=nn;
                  for(int i=0;i<n;i++) zn(i,nn)=z(i);
                  s(nn++)=newdist;
               }
               else 
               {
                  if(newdist<s(imax)) 
                  {
                     for(int i=0;i<n;i++) zn(i,imax)=z(i);
                     s(imax)=newdist;
                     for(int i=imax=0;i<m;i++) if (s(imax)<s(i)) imax=i;
                  }
                  maxdist=s(imax);
               }
               z(0)+=step(0); y=zb(0)-z(0); step(0)=-step(0)-sign(step(0));
            }
         }
         else 
         {
            if(k==n-1) break;
            else 
            {
               k++;
               z(k)+=step(k); y=zb(k)-z(k); step(k)=-step(k)-sign(step(k));
            }
         }
      }
      for(int i=0;i<m-1;i++) 
      { 
         for(int j=i+1;j<m;j++) 
         {
            if(s(i)<s(j)) continue;
            swap(s(i),s(j));
            for(k=0;k<n;k++) swap(zn(k,i),zn(k,j));
         }
      }

      if (c>=LOOPMAX) 
      {
         return -1;
      }

      return 0;
   
   }  // End of method 'ARMLambda::search()'
   

}   // End of namespace gpstk

