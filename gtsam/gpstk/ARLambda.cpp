#pragma ident "$Id$"

/**
 * @file ARLambda.cpp
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

#include "ARLambda.hpp"


namespace gpstk
{
   Vector<double> ARLambda::resolveIntegerAmbiguity( 
                                               const Vector<double>& ambFloat, 
                                               const Matrix<double>& ambCov )
      throw(ARException)
   {
      // check input
      if( ambFloat.size()!=ambCov.rows() || ambFloat.size()!=ambCov.cols() )
      {
         ARException e("The dimension of input does not match.");
         GPSTK_THROW(e);
      }

      try
      {
         Matrix<double> F; Vector<double> S;
         if( lambda(ambFloat,ambCov,F,S,2)==0 )
         {
            Vector<double> ambFixed( ambFloat.size(), 0.0 );
            for(size_t i=0; i<ambFloat.size(); i++) 
            {
               ambFixed(i) = F(i,0);
            }

            squaredRatio = (S(0)<1e-12) ? 9999.9 : S(1)/S(0);
            
            return ambFixed;
         }
         else
         {
            ARException e("Failed to resolve the integer ambiguities.");
            GPSTK_THROW(e);
         }
      }
      catch(Exception& e)
      {
         GPSTK_RETHROW(e);
      }
      catch (...)
      {
         ARException e("Failed to resolve the integer ambiguities.");
         GPSTK_THROW(e);
      }
         // it should never go here, but we sill return the float ambiguities
      return ambFloat;

   }  // End of method 'ARMLambda::resolveIntegerAmbiguity()'

   
   int ARLambda::factorize( const Matrix<double>& Q, 
                            Matrix<double>& L, 
                            Vector<double>& D)
   {
      // TODO: CHECK UNEXPECTED INPUT
      // L:nxn Z:nxn 0<=(i,j)<n

      const int n = static_cast<int>(Q.rows());

      Matrix<double> QC(Q);
      L.resize(n, n, 0.0);
      D.resize(n, 0.0);

      for(int i = n-1; i >= 0; i--) 
      {
         D(i) = QC(i,i);
         if( D(i) <= 0.0 ) return -1;
         double temp = std::sqrt(D(i));
         for(int j=0; j<=i; j++) L(i,j) = QC(i,j)/temp;
         for(int j=0; j<=i-1; j++) 
         {
            for(int k=0; k<=j; k++) QC(j,k) -= L(i,k) * L(i,j);
         }
         for(int j=0; j<=i; j++) L(i,j) /= L(i,i);
      }

      return 0;

   }  // End of method 'ARLambda::factorize()'


   void ARLambda::gauss(Matrix<double>& L, Matrix<double>& Z, int i, int j)
   {
      //TODO: CHECK UNEXPECTED INPUT
      // L:nxn Z:nxn 0<=(i,j)<n

      const int n = L.rows();
      const int mu = (int)round(L(i,j));
      if(mu != 0) 
      {
         for(int k=i; k<n; k++) L(k,j) -= (double)mu*L(k,i);
         for(int k=0; k<n; k++) Z(k,j) -= (double)mu*Z(k,i);
      }
   }  // End of method 'ARLambda::gauss()'

   
   void ARLambda::permute( Matrix<double>& L, 
                           Vector<double>& D, 
                           int j, 
                           double del, 
                           Matrix<double>& Z )
   {  
      //TODO: CHECK UNEXPECTED INPUT
      // L:nxn D:nx1 Z:nxn 0<=j<n

      const int n = L.rows();

      double eta=D(j)/del;
      double lam=D(j+1)*L(j+1,j)/del;

      D[j]=eta*D(j+1); 
      D(j+1)=del;
      for(int k=0;k<=j-1;k++) 
      {
         double a0=L(j,k); 
         double a1=L(j+1,k);
         L(j,k) =-L(j+1,j)*a0 + a1;
         L(j+1,k) = eta*a0 + lam*a1;
      }
      L(j+1,j)=lam;
      for(int k=j+2; k<n; k++) swap(L(k,j),L(k,j+1));
      for(int k=0; k<n; k++) swap(Z(k,j),Z(k,j+1));

   }  // End of method 'ARLambda::permute()'

   
   void ARLambda::reduction( Matrix<double>& L, 
                             Vector<double>& D,
                             Matrix<double>& Z)
   {
      //TODO: CHECK UNEXPECTED INPUT
      // L:nxn D:nx1 Z:nxn 0<=j<n

      const int n = L.rows();

      int j(n-2), k(n-2);
      while(j>=0) 
      {
         if(j<=k)
         {
            for (int i=j+1; i<n; i++) 
            {
               gauss(L,Z,i,j);
            }
         } 

         double del=D(j)+L(j+1,j)*L(j+1,j)*D(j+1);

         if(del+1E-6<D(j+1)) 
         { 
            permute(L,D,j,del,Z);
            k=j; j=n-2;
         }
         else
         { 
            j--;
         }

      }  // end while

   }  // End of method 'ARLambda::reduction()'


   int ARLambda::search( const Matrix<double>& L, 
                         const Vector<double>& D, 
                         const Vector<double>& zs, 
                         Matrix<double>& zn, 
                         Vector<double>& s, 
                          const int& m )
   {

      ARException e("The LAMBDA search method have not been implemented.");
      GPSTK_THROW(e);

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

   }  // End of method 'ARLambda::search()'


   int ARLambda::lambda( const Vector<double>& a, 
                         const Matrix<double>& Q, 
                         Matrix<double>& F,
                         Vector<double>& s,
                         const int& m )
   {
      if( (a.size()!=Q.rows()) || (Q.rows()!=Q.cols()) ) return -1;
      if( m < 1) return -1;

      const int n = static_cast<int>(a.size());

      Matrix<double> L(n,n,0.0),E(n,m,0.0);
      Vector<double> D(n,0.0),z(n,0.0);
      Matrix<double> Z = ident<double>(n);

      if (factorize(Q,L,D)==0) 
      {      
         reduction(L,D,Z);
         z = transpose(Z)*a;

         if (search(L,D,z,E,s,m)==0) 
         {
            try
            {  // F=Z'\E - Z nxn  E nxm F nxm
               F = transpose( inverseLUD(Z) ) * E;
            }
            catch(...) 
            { return -1; }
         }
      }

      return 0;

   }  // End of method 'ARLambda::lambda()'


}   // End of namespace gpstk

