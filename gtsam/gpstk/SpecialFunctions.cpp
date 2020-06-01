#pragma ident "$Id$"

/**
 * @file SpecialFunctions.cpp
 * Contains handy special functions: Gamma, erf, erfc, etc.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008
//
//============================================================================


#include "SpecialFunctions.hpp"


using namespace std;

namespace gpstk
{


      /* Computes the Gamma function using a simple Lanczos approximation.
       *
       * This implementation typically gives 15 correct decimal places, and
       * it is adapted from free Python code found in:
       *
       * http://en.wikipedia.org/wiki/Lanczos_approximation
       *
       * \warning Be aware that Gamma function is not defined for 0, -1, -2,...
       */
   double gamma(double val)
   {

      double inf( 9.0e+99 );

      if( val == 0.0 )
      {
         return inf;
      }

      if ( ( val < 0.0 ) &&
           ( floor(val) == val ) )
      {
         return inf;
      }

         // Set the number of coefficients being used
      int g(7);
      const double lanczos_coef[] = { 0.99999999999980993,
                                      676.5203681218851,
                                      -1259.1392167224028,
                                      771.32342877765313,
                                      -176.61502916214059,
                                      12.507343278686905,
                                      -0.13857109526572012,
                                      9.9843695780195716e-6,
                                      1.5056327351493116e-7 };

      if(val < 0.5)
      {
         return ( PI / (sin(PI*val)*gamma(1.0-val)) );
      }
      else
      {
         val -= 1.0;

         double x( lanczos_coef[0] );

         for(int i = 1; i<g+2; i++)
         {
            x += lanczos_coef[i]/(val+(double)i);
         }

         double t (val + static_cast<double>(g) + 0.5);

         return ( 2.5066282746310002 * pow( t, (val+0.5) ) * exp(-t) * x );

      }

   }  // End of function 'gamma()'



      /* Computes the natural logarithm of Gamma function
       * using the Lanczos approximation.
       *
       * \warning This version does not work for values <= 0.0
       */
   double lngamma(double val)
   {

      double inf( 9.0e+99 );

      if( val <= 0.0 )
      {
         return inf;
      }

         // Set the number of coefficients being used
      int g(7);
      const double lanczos_coef[] = { 0.99999999999980993,
                                      676.5203681218851,
                                      -1259.1392167224028,
                                      771.32342877765313,
                                      -176.61502916214059,
                                      12.507343278686905,
                                      -0.13857109526572012,
                                      9.9843695780195716e-6,
                                      1.5056327351493116e-7 };

      if(val < 0.5)
      {
         return ( 1.1447298858494002 - (log(sin(PI*val)) + lngamma(1.0-val)) );
      }
      else
      {
         val -= 1.0;

         double x( lanczos_coef[0] );

         for(int i = 1; i<g+2; i++)
         {
            x += lanczos_coef[i]/(val+(double)i);
         }

         double t (val + static_cast<double>(g) + 0.5);

         return ( 0.918938533204672741781 + (val+0.5)*log(t) + (-t) + log(x) );
      }


   }  // End of function 'lngamma()'



      // Auxiliar Kummer function.
   double kummerFunc(const double a, const double z);



      // We compute the lower incomplete gamma function g(a,z) using the
      // formula:
      //
      //        g(a,z) = z**a * exp(-z) * S(a,z) / a
      //
      // where:
      //
      //                  oo
      //                 ___            k
      //                \              z
      //   S(a,z) = 1 +  )     ------------------.
      //                /___   (a+1)(a+2)...(a+k)
      //                k = 1
      //
      // S(a,z) is computed with the Kummer function "kummerFunc()".
   double kummerFunc(const double aval, const double zval)
   {

      double eps(1.0e-15);    // Small threshold controling precision

      double z( abs(zval) );    // We only allow positive values of 'z'
      double a( abs(aval) );    // We only allow positive values of 'a'

      double den(a);          // Variable to store denominator

      double s(1.0);          // Result will be stored here

      double coef(1.0);       // Initialize coefficient

      while( abs(coef) > eps)
      {
         coef = coef*z;   // Compute numerator
         den += 1.0;
         coef = coef/den;  // Compute coefficient
         s += coef;        // Add new coefficient to result
      }

      return s;

   }  // End of function 'kummerFunc()'



      // Lower incomplete gamma function.
   double lower_gamma(const double a, const double z)
   {

      double zp( abs(z) );    // We only allow positive values of 'z'
      double ap( abs(a) );    // We only allow positive values of 'a'

      double s( kummerFunc(ap, zp) );

      return exp(log(zp)*ap) * exp(-zp) * s / ap;

   }  // End of function 'lower_gamma()'



      // Upper incomplete gamma function.
   double upper_gamma(const double a, const double z)
   {

      return ( gamma(a) - lower_gamma(a, z) );

   }  // End of function 'upper_gamma()'



      // Lower incomplete regularized gamma function P(a,z).
   double gammaP(const double a, const double z)
   {
      return ( lower_gamma(a,z) / gamma(a) );
   }



      // Upper incomplete regularized gamma function Q(a,z).
   double gammaQ(const double a, const double z)
   {
      return ( 1.0 - gammaP(a,z) );
   }



      /* Computes factorial of integer number n.
       *
       * This implementation typically gives 15 correct decimal places, and
       * returns the result as double.
       */
   double factorial(const int n)
   {

         // Return 0 if n<0
      if( n < 0 ) return 0.0;

      double facttable[] = { 1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0,
                             40320.0, 362880.0, 3628800.0, 39916800.0,
                             479001600.0, 6227020800.0, 87178291200.0,
                             1307674368000.0 };

         // Use a look-up table for small values of n
      if( (n >= 0) && (n <= 15) )
      {
         return facttable[n];
      }

         // Use gamma function properties for big values of n
      return gamma(n+1.0);

   }

	    /* Computes factorial of double number d.
       */
   double factorial(const double d)
   {
      	// copy the input
      double n(d);

      double returnValue(0.0);

      if (n < 0.0) 
      {
         return returnValue;

         // Exception e("Illegal value: ");
         // GPSTK_THROW(e);
      } 
      else if ((n == 0.0)||(n == 1.0)) 
      {
         returnValue = 1.0;

      } else	
      {
         returnValue = n * factorial(n - 1.0);
      }

      return returnValue;
   }



      // Auxiliar error function #1. erf(x) for x in [ 0, 0.84375 ]
   double erf1(const double x);


      // Auxiliar error function #2. erf(x) for x in [ 0.84375, 1.25 ]
   double erf2(const double x);


      // Auxiliar error function #3. erf(x) for x in [ 1.25, 2.857142 ]
   double erf3(const double x);


      // Auxiliar error function #4. erf(x) for x in [ 2.857142, 6.0 ]
   double erf4(const double x);


      // Auxiliar error function #5. erf(x) for x in [ 6.0, inf ]
   double erf5(const double x);



      /* Error function.
       *
       * This is a C++ implementation of the free Python code found in:
       *
       *   http://code.activestate.com/recipes/576391/
       *
       * Such code was based in a C code base with OpenBSD license from:
       *
       * ====================================================
       * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
       *
       * Developed at SunPro, a Sun Microsystems, Inc. business.
       * Permission to use, copy, modify, and distribute this
       * software is freely granted, provided that this notice
       * is preserved.
       * ====================================================
       */
   double erf(const double x)
   {

      /*
       *                            |x
       *                    2       |\
       *     erf(x)  =  ---------   |   exp(-t*t)dt
       *                 sqrt(pi)  \|
       *                            |0
       *
       *     erfc(x) =  1-erf(x)
       *  Note that
       *              erf(-x) = -erf(x)
       *             erfc(-x) = 2 - erfc(x)
       *
       * Method:
       *      1. For |x| in [0, 0.84375]
       *          erf(x)  = x + x*R(x^2)
       *          erfc(x) = 1 - erf(x)           if x in [-.84375,0.25]
       *                  = 0.5 + ((0.5-x)-x*R)  if x in [0.25,0.84375]
       *
       *         where R = P/Q where P is an odd poly of degree 8 and
       *         Q is an odd poly of degree 10.
       *
       *                      | R - (erf(x)-x)/x | <= 2^(-57.90)
       *
       *
       *         Remark. The formula is derived by noting
       *            erf(x) = (2/sqrt(pi))*(x - x^3/3 + x^5/10 - x^7/42 + ....)
       *         and that
       *            2/sqrt(pi) = 1.128379167095512573896158903121545171688
       *         is close to one. The interval is chosen because the fix
       *         point of erf(x) is near 0.6174 (i.e., erf(x)=x when x is
       *         near 0.6174), and by some experiment, 0.84375 is chosen to
       *         guarantee the error is less than one ulp for erf.
       *
       *      2. For |x| in [0.84375,1.25], let s = |x| - 1, and
       *         c = 0.84506291151 rounded to single (24 bits)
       *              erf(x)  = sign(x) * (c  + P1(s)/Q1(s))
       *              erfc(x) = (1-c)  - P1(s)/Q1(s) if x > 0
       *                        1+(c+P1(s)/Q1(s))    if x < 0
       *              |P1/Q1 - (erf(|x|)-c)| <= 2^(-59.06)
       *
       *         Remark: here we use the taylor series expansion at x=1.
       *              erf(1+s) = erf(1) + s*Poly(s)
       *                       = 0.845.. + P1(s)/Q1(s)
       *         That is, we use rational approximation to approximate
       *              erf(1+s) - (c = (single)0.84506291151)
       *         Note that |P1/Q1|< 0.078 for x in [0.84375,1.25]
       *         where
       *              P1(s) = degree 6 poly in s
       *              Q1(s) = degree 6 poly in s
       *
       *      3. For x in [1.25,1/0.35(~2.857143)],
       *              erfc(x) = (1/x)*exp(-x*x-0.5625+R1/S1)
       *              erf(x)  = 1 - erfc(x)
       *         where
       *              R1(z) = degree 7 poly in z, (z=1/x^2)
       *              S1(z) = degree 8 poly in z
       *
       *      4. For x in [1/0.35,28]
       *              erfc(x) = (1/x)*exp(-x*x-0.5625+R2/S2) if x > 0
       *                      = 2.0 - (1/x)*exp(-x*x-0.5625+R2/S2) if -6<x<0
       *                      = 2.0 - tiny      (if x <= -6)
       *              erf(x)  = sign(x)*(1.0 - erfc(x)) if x < 6, else
       *              erf(x)  = sign(x)*(1.0 - tiny)
       *         where
       *              R2(z) = degree 6 poly in z, (z=1/x^2)
       *              S2(z) = degree 7 poly in z
       *
       *         Note1:
       *            To compute exp(-x*x-0.5625+R/S), let s be a single
       *            precision number and s := x; then
       *              -x*x = -s*s + (s-x)*(s+x)
       *              exp(-x*x-0.5626+R/S) =
       *                               exp(-s*s-0.5625)*exp((s-x)*(s+x)+R/S);
       *
       *         Note2:
       *            Here 4 and 5 make use of the asymptotic series
       *                         exp(-x*x)
       *              erfc(x) ~ ---------- * ( 1 + Poly(1/x^2) )
       *                        x*sqrt(pi)
       *            We use rational approximation to approximate
       *              g(s)=f(1/x^2) = log(erfc(x)*x) - x*x + 0.5625
       *            Here is the error bound for R1/S1 and R2/S2
       *              |R1/S1 - f(x)|  < 2**(-62.57)
       *              |R2/S2 - f(x)|  < 2**(-61.52)
       *
       *      5. For inf > x >= 28
       *              erf(x)  = sign(x) *(1 - tiny)  (raise inexact)
       *              erfc(x) = tiny*tiny (raise underflow) if x > 0
       *                      = 2 - tiny if x<0
       *
       *      7. Special case:
       *              erf(0)  = 0, erf(inf)  = 1, erf(-inf) = -1,
       *              erfc(0) = 1, erfc(inf) = 0, erfc(-inf) = 2,
       *              erfc/erf(NaN) is NaN
       */

      double inf( 9e+99 );

       if (x >= inf)
      {
         return 1.0;
      }

      if (x <= -inf)
      {
         return -1.0;
      }

      if ( fabs(x) < 0.84375 )
      {
         return erf1(x);
      }
      else if ( ( 0.84375 <= fabs(x) ) &&
                ( fabs(x) < 1.25 ) )
      {
         return erf2(x);
      }
      else if ( ( 1.25 <= fabs(x) ) &&
                ( fabs(x) < 2.857142) )
      {
         return erf3(x);
      }
      else if ( ( 2.857142 <= fabs(x) ) &&
                ( fabs(x) < 6.0 ) )
      {
         return erf4(x);
      }

         // If we got here, it means that ( fabs(x) >= 6.0 )
      return erf5(x);

   }  // End of function 'erf()'



      // Auxiliar error function #1. erf(x) for x in [0,0.84375].
   double erf1(const double x)
   {

      int i;

         // Get the base-2 exponent of input 'x'
      frexp (x , &i);

      if ( fabs( static_cast<double>(i) ) > 28.0 )
      {
         if( fabs( static_cast<double>(i) ) > 57.0 )
         {
            return ( 1.128379167095512586316e+0 * x );
         }

         return ( x * ( 2.28379167095512586316e-01 ) );
      }

      double z( x*x );
      double r( 1.28379167095512558561e-01
               + z * ( -3.25042107247001499370e-01
               + z * ( -2.84817495755985104766e-02
               + z * ( -5.77027029648944159157e-03
               + z * -2.37630166566501626084e-05 ) ) ) );


      double s( 1.0 + z * ( 3.97917223959155352819e-01
                    + z * ( 6.50222499887672944485e-02
                    + z * ( 5.08130628187576562776e-03
                    + z * ( 1.32494738004321644526e-04
                    + z * -3.96022827877536812320e-06 ) ) ) ) );

      return ( x * (1.0 + r/s ) );

   }  // End of function 'erf1()'



      // Auxiliar error function #2. erf(x) for x in [0.84375,1.25]
   double erf2(const double x)
   {

      double s( fabs(x) - 1.0 );

      double P( -2.36211856075265944077e-03
               + s * ( 4.14856118683748331666e-01
               + s * ( -3.72207876035701323847e-01
               + s * ( 3.18346619901161753674e-01
               + s * ( -1.10894694282396677476e-01
               + s * ( 3.54783043256182359371e-02
               + s * -2.16637559486879084300e-03 ) ) ) ) ) );

      double Q( 1.0 + s * ( 1.06420880400844228286e-01
                    + s * ( 5.40397917702171048937e-01
                    + s * ( 7.18286544141962662868e-02
                    + s * ( 1.26171219808761642112e-01
                    + s * ( 1.36370839120290507362e-02
                    + s * 1.19844998467991074170e-02 ) ) ) ) ) );

      if( x >= 0.0 )
      {
         return ( 8.45062911510467529297e-01 + P/Q );
      }
      else
      {
         return ( -8.45062911510467529297e-01 - P/Q );
      }

   }  // End of function 'erf2()'



      // Auxiliar error function #3. erf(x) for x in [1.25,2.857142]
   double erf3(const double xval)
   {

      double x0(xval);
      double x( fabs(xval) );
      double s( 1.0/(x*x) );

      double R( -9.86494403484714822705e-03
               + s * ( -6.93858572707181764372e-01
               + s * ( -1.05586262253232909814e+01
               + s * ( -6.23753324503260060396e+01
               + s * ( -1.62396669462573470355e+02
               + s * ( -1.84605092906711035994e+02
               + s * ( -8.12874355063065934246e+01
               + s * -9.81432934416914548592e+00 ) ) ) ) ) ) );

      double S( 1.0 + s * ( 1.96512716674392571292e+01
                    + s * ( 1.37657754143519042600e+02
                    + s * ( 4.34565877475229228821e+02
                    + s * ( 6.45387271733267880336e+02
                    + s * ( 4.29008140027567833386e+02
                    + s * ( 1.08635005541779435134e+02
                    + s * ( 6.57024977031928170135e+00
                    + s * -6.04244152148580987438e-02 ) ) ) ) ) ) ) );

      double r( exp(-x0*x0-0.5625) * exp( (x0-x)*(x0+x)+R/S) );

      if( x0 >= 0.0 )
      {
         return ( 1.0 - r/x );
      }
      else
      {
         return ( r/x - 1.0 );
      }

   }  // End of function 'erf3()'



      // Auxiliar error function #4. erf(x) for x in [ 2.857142, 6.0 ]
   double erf4(const double xval)
   {

      double x0(xval);
      double x( fabs(xval) );
      double s( 1.0/(x*x) );

      double R( -9.86494292470009928597e-03
               + s * ( -7.99283237680523006574e-01
               + s * ( -1.77579549177547519889e+01
               + s * ( -1.60636384855821916062e+02
               + s * ( -6.37566443368389627722e+02
               + s * ( -1.02509513161107724954e+03
               + s * -4.83519191608651397019e+02 ) ) ) ) ) );

      double S( 1.0 + s * ( 3.03380607434824582924e+01
                    + s * ( 3.25792512996573918826e+02
                    + s * ( 1.53672958608443695994e+03
                    + s * ( 3.19985821950859553908e+03
                    + s * ( 2.55305040643316442583e+03
                    + s * ( 4.74528541206955367215e+02
                    + s * -2.24409524465858183362e+01 ) ) ) ) ) ) );

      double r( exp( -x0 * x0 - 0.5625 ) * exp( (x0-x)*(x0+x) + R/S ) );

      if( x0 >= 0.0 )
      {
         return ( 1.0 - r/x );
      }
      else
      {
         return ( r/x - 1.0 );
      }

   }  // End of function 'erf4()'



      // Auxiliar error function #5. erf(x) for x in [ 6.0, inf ]
   double erf5(const double x)
   {

      double tiny( 1e-99 );

      if ( x > 0.0 )
      {
         return ( 1.0 - tiny );
      }
      else
      {
         return ( tiny - 1.0 );
      }

   }  // End of function 'erf5()'



      // Complementary error function.
   double erfc(const double x)
   {

      return ( 1.0 - erf(x) );

   }  // End of function 'erfc()'



      /* Inverse of error function.
       *
       * \ warning Value "z" must be in the range (-1, 1)
       */
   double inverf(const double z)
   {

      double inf( 9.0e+99 );

         // Check limits
      if( z >= 1.0 ) return inf;
      if( z <= -1.0 ) return -inf;

      double z2PI( z*z*PI );

         // Compute the approximation found in:
         //    http://en.wikipedia.org/wiki/Error_function
         // The module of this approximation is always under fabs(inverf())
      double x( 0.88622692545275794 * z
              * ( 1.0 + z2PI * ( 0.083333333333333333
                      + z2PI * ( 0.014583333333333334
                      + z2PI * ( 0.0031498015873015874
                      + z2PI * ( 0.00075248704805996468
                      + z2PI * ( 0.0001907475361251403
                      + z2PI * ( 1.8780048076923078e-5
                      + z2PI * ( 1.3623642420578133e-5 ) ) ) ) ) ) ) ) );

      double delta(1.0);
      double threshold( 1.0e-10 );
      int iter( 0 );

      while ( ( fabs(delta) > threshold ) &&
              ( iter < 100 ) )
      {

            // Use the Newton-Raphson method. Denominator is d(erf(x))/dx
         delta = (z - gpstk::erf(x)) / (1.1283791670955126*std::exp(-x*x));

         x = x + delta;

         ++iter;

      }

      return x;

   }  // End of function 'inverf()'



      /* Beta function.
       *
       * \warning This version may not work for values > 130.0
       */
   double beta(const double x, const double y)
   {

      return ( gamma(x) * gamma(y) / gamma( x + y ) );

   }  // End of function 'beta()'



      /* Computes the natural logarithm of Beta function
       *
       * \warning This version does not work for values <= 0.0
       */
   double lnbeta(double x, double y)
   {

      x = fabs(x);
      y = fabs(y);

      return ( lngamma(x) + lngamma(y) - lngamma( x + y ) );

   }  // End of function 'lbeta()'



      /* Auxiliar function to compute incomplete beta function.
       * Power series for incomplete beta integral.
       * Use when b*x is small and x not too close to 1.
       */
   double incompletebetaps(const double x, const double a, const double b)
   {

         // Declare working variables
      double epsilon(1.0e-30);
      double big(1.0e99);
      double small(1.0e-99);
      double maxgam(171.0);   // Approximate maximum input for gamma function
      double ai( 1.0/a );
      double u( (1.0-b) * x );
      double v( u / (a+1.0) );
      double t1(v);
      double t(u);
      double n(2.0);
      double s(0.0);
      double z( epsilon*ai );

      while( std::fabs(v) > z )
      {
         u = (n-b) * x / n;
         t = t * u;
         v = t/(a+n);
         s = s+v;
         n = n+1.0;
      }

      s = s + t1 + ai;
      u = a * std::log(x);

         // Check if we can compute gamma values
      if( (a+b < maxgam) &&
          ( std::fabs(u) < std::log(big) ) )
      {

         t = gamma(a+b) / ( gamma(a) * gamma(b) );
         s = s * t * pow(x, a);

      }
      else
      {

         t = lngamma(a+b) - lngamma(a)- lngamma(b) + u + std::log(s);

         if( t < std::log(small) )
         {
            s = 0.0;
         }
         else
         {
            s = std::exp(t);
         }

      }  // End of block 'if( (a+b < maxgam) && ( std::fabs(u) ...'

      return s;

   }  // End of function 'incompletebetaps()'



      // Auxiliar function to compute incomplete beta function.
   double incompletebetafe(const double x, const double a, const double b)
   {

         // Declare auxiliar parameters
      double epsilon(1.0e-30);
      double big( 1.0e16 );
      double biginv( 1.0/big );

      double k1( a );
      double k2( a+b );
      double k3( a );
      double k4( a+1.0 );
      double k5( 1.0 );
      double k6( b-1.0 );
      double k7( k4 );
      double k8( a+2.0 );

      double pkm1( 1.0 );
      double pkm2( 0.0 );
      double qkm1( 1.0 );
      double qkm2( 1.0 );

      double ans( 1.0 );
      double r( 1.0 );
      double thresh( 3.0 * epsilon );

      int n( 0 );

         // Start iteration
      do
      {

         double xk( -x * k1 * k2 / (k3*k4) );
         double pk( pkm1 + pkm2 * xk );
         double qk( qkm1 + qkm2 * xk );

         pkm2 = pkm1;
         pkm1 = pk;
         qkm2 = qkm1;
         qkm1 = qk;

         xk = x * k5 * k6 / (k7*k8);
         pk = pkm1 + pkm2 * xk;
         qk = qkm1 + qkm2 * xk;

         pkm2 = pkm1;
         pkm1 = pk;
         qkm2 = qkm1;
         qkm1 = qk;

         if( qk != 0.0 )
         {
            r = pk/qk;
         }

         double t;

         if( r != 0.0 )
         {
            t = std::fabs( (ans-r)/r );
            ans = r;
         }
         else
         {
            t = 1.0;
         }

            // Exit if we are under threshold
         if( t < thresh )
         {
            break;
         }

            // Recompute auxiliar parameters
         k1 = k1 + 1.0;
         k2 = k2 + 1.0;
         k3 = k3 + 2.0;
         k4 = k4 + 2.0;
         k5 = k5 + 1.0;
         k6 = k6 - 1.0;
         k7 = k7 + 2.0;
         k8 = k8 + 2.0;

            // Check if qk, pk are too big
         if( (std::fabs(qk) + std::fabs(pk)) > big )
         {
            pkm2 = pkm2 * biginv;
            pkm1 = pkm1 * biginv;
            qkm2 = qkm2 * biginv;
            qkm1 = qkm1 * biginv;
         }

            // Check if qk, pk are too small
         if( (std::fabs(qk) < biginv) ||
             (std::fabs(pk) < biginv ) )
         {
            pkm2 = pkm2 * big;
            pkm1 = pkm1 * big;
            qkm2 = qkm2 * big;
            qkm1 = qkm1 * big;
         }

            // Increase n
         n = n+1;

      }
      while( n < 300 );

         // Return result
      return ans;

   }  // End of function 'incompletebetafe()'



      // Auxiliar function to compute incomplete beta function.
   double incompletebetafe2(const double x, const double a, const double b)
   {

         // Declare auxiliar parameters
      double epsilon(1.0e-30);
      double big( 1.0e16 );
      double biginv( 1.0/big );

      double k1( a );
      double k2( b-1.0 );
      double k3( a );
      double k4( a+1.0 );
      double k5( 1.0 );
      double k6( a+b );
      double k7( a+1.0 );
      double k8( a+2.0 );

      double pkm1( 1.0 );
      double pkm2( 0.0 );
      double qkm1( 1.0 );
      double qkm2( 1.0 );

      double z( x / (1.0-x) );
      double ans( 1.0 );
      double r( 1.0 );
      double thresh( 3.0 * epsilon );

      int n( 0 );

         // Start iteration
      do
      {

         double xk( -z * k1 * k2 / (k3*k4) );
         double pk( pkm1 + pkm2 * xk );
         double qk( qkm1 + qkm2 * xk );

         pkm2 = pkm1;
         pkm1 = pk;
         qkm2 = qkm1;
         qkm1 = qk;

         xk = z * k5 * k6 / (k7*k8);
         pk = pkm1 + pkm2 * xk;
         qk = qkm1 + qkm2 * xk;

         pkm2 = pkm1;
         pkm1 = pk;
         qkm2 = qkm1;
         qkm1 = qk;

         if( qk != 0.0 )
         {
            r = pk/qk;
         }

         double t;

         if( r != 0.0 )
         {
            t = std::fabs( (ans-r) / r );
            ans = r;
         }
         else
         {
            t = 1.0;
         }

            // Exit if we are under threshold
         if( t < thresh )
         {
            break;
         }


            // Recompute auxiliar parameters
         k1 = k1 + 1.0;
         k2 = k2 - 1.0;
         k3 = k3 + 2.0;
         k4 = k4 + 2.0;
         k5 = k5 + 1.0;
         k6 = k6 + 1.0;
         k7 = k7 + 2.0;
         k8 = k8 + 2.0;

            // Check if qk, pk are too big
         if( (std::fabs(qk) + std::fabs(pk)) > big )
         {
            pkm2 = pkm2 * biginv;
            pkm1 = pkm1 * biginv;
            qkm2 = qkm2 * biginv;
            qkm1 = qkm1 * biginv;
         }

            // Check if qk, pk are too small
         if( (std::fabs(qk) < biginv) ||
             (std::fabs(pk) < biginv ) )
         {
            pkm2 = pkm2 * big;
            pkm1 = pkm1 * big;
            qkm2 = qkm2 * big;
            qkm1 = qkm1 * big;
         }

            // Increase n
         n = n+1;

      }
      while( n < 300 );

         // Return result
      return ans;

   }  // End of function 'incompletebetafe2()'



      /* Computes the regularized incomplete Beta function Ix(a,b).
       *
       * This code is a C++ implementation and adaptation from code found
       * in Cephes Math Library Release 2.8, copyright by Stephen L. Moshier,
       * released under a BSD license.
       */
   double regIncompleteBeta(const double x, const double a, const double b)
      throw(InvalidParameter)
   {

         // Let's do some basic checks
      if( (a <= 0.0) || (b <= 0.0) )
      {
         InvalidParameter e("Function 'regIncompleteBeta()': 'a' and 'b' must \
be greater than zero." );
         GPSTK_THROW(e);
      }

      if( (x < 0.0) || (x > 1.0) )
      {
         InvalidParameter e("Function 'regIncompleteBeta()': 'x' must be \
within the interval [0,1]." );
         GPSTK_THROW(e);
      }

      if (x == 0.0) return 0.0;
      if (x == 1.0) return 1.0;

         // Declare working parameters
      int flag(0);
      double epsilon(1.0e-30);
      double xx(x);
      double aa(a);
      double bb(b);


         // Check if bb*xx is small and xx not too close to 1.
      if( (bb*xx <= 1.0) &&
          (xx <= 0.95) )
      {
         return incompletebetaps(xx, aa, bb);
      }

      double w( 1.0-xx );
      double t, xc;

      if( xx > (aa/(aa+bb)) )
      {
         flag = 1;
         t = aa;
         aa = bb;
         bb = t;
         xc = xx;
         xx = w;
      }
      else
      {
         xc = w;
      }

      double result(0.0);

      if( (flag==1)      &&
          (bb*xx <= 1.0) &&
          (xx <= 0.95) )
      {

            // bb*xx is small and xx not too close to 1.
         t = incompletebetaps(xx, aa, bb);

         if( t <= epsilon )
         {
            result = 1.0 - epsilon;
         }
         else
         {
            result = 1.0 - t;
         }

         return result;

      }

      double y( xx * (aa+bb-2.0) - (aa-1.0) );

      if( y < 0.0 )
      {
         w = incompletebetafe(xx, aa, bb);
      }
      else
      {
         w = incompletebetafe2(xx, aa, bb) / xc;
      }

      t = std::pow(xc, bb);
      t = t * std::pow(xx, aa);
      t = t/aa;
      t = t*w;
      t = t * ( gamma(aa+bb) / ( gamma(aa) * gamma(bb) ) );

      if( flag==1 )
      {
         if( t <= epsilon )
         {
            result = 1.0 - epsilon;
         }
         else
         {
            result = 1.0 - t;
         }
      }
      else
      {
         result = t;
      }

      return result;

   }  // End of function 'incompleteBeta()'



}  // End of namespace gpstk
