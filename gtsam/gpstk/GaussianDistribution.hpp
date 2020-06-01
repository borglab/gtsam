#pragma ident "$Id$"

/**
 * @file GaussianDistribution.hpp
 * This class implements the Gaussian (or normal) distribution.
 */

#ifndef GAUSSIANDISTRIBUTION_HPP
#define GAUSSIANDISTRIBUTION_HPP

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


#include "BaseDistribution.hpp"


namespace gpstk
{

      /** @addtogroup math */
      //@{

      /** This class implements the Gaussian (or normal) distribution.
       *
       * A typical way to use this class follows:
       *
       * @code
       *
       *      // Declare a  'GaussianDistribution' object, with nominal values
       *   GaussianDistribution normal;
       *
       *      // Declare another, non-nominal 'GaussianDistribution' object.
       *   GaussianDistribution gauss(1.341, 0.176);
       *
       *      // Print results for several values
       *   for(int i = 0; i < 1601; i++)
       *   {
       *
       *      double x( -8.0 + ( static_cast<double>(i)/100.0 ) );
       *
       *      cout << x << " | "
       *           << normal.pdf(x) << " | "
       *           << gauss.pdf(x) << " | "
       *           << normal.Q(x) << " | "
       *           << gauss.Q(x) << endl;
       *
       *   }
       *
       * @endcode
       *
       * @sa SpecialFunctions.hpp for useful functions, and
       *     Chi2Distribution.hpp for a Chi-square distribution.
       *
       */
   class GaussianDistribution : public BaseDistribution
   {
   public:


         /** Default constructor. Sets a standard normal distribution:
          *  mean = 0.0, and standard deviation = 1.0.
          */
      GaussianDistribution();


         /** Explicit constructor.
          *
          * @param mu      Mean
          * @param sig     Standard deviation
          *
          * \warning If (sig <= 0.0), it will be set to 1.0.
          */
      GaussianDistribution( double mu,
                            double sig );


         /** Computes the probability density function
          *
          * @param x    Value
          */
      virtual double pdf(double x);


         /** Computes the cumulative distribution function
          *
          * @param x    Value
          */
      virtual double cdf(double x);


         /** Computes the upper tail of the Gaussian probability
          *  function Q(x).
          *
          * @param x    Value
          */
      virtual double Q(double x)
      { return ( 1.0 - cdf(x) ); };


         /** Computes the quantile function ( cdf^-1() )
          *
          * @param p    Probability value
          *
          * \ warning Value "p" must be in the range (0, 1)
          */
      virtual double invcdf(double p)
         throw(InvalidParameter);


         /// Get mean
      virtual double getMean(void) const
      { return mean; };


         /** Sets the mean
          *
          * @param mu      Mean
          */
      virtual GaussianDistribution& setMean(double mu)
      { mean = mu; recompute(); return (*this); };


         /// Get standard deviation
      virtual double getSigma(void) const
      { return sigma; };


         /** Sets the standard deviation
          *
          * @param sig     Standard deviation
          *
          * \warning If (sig <= 0.0), it will be set to 1.0.
          */
      virtual GaussianDistribution& setSigma(double sig);


         /** Sets all parameters in one pass.
          *
          * @param mu      Mean
          * @param sig     Standard deviation
          *
          * \warning If (sig <= 0.0), it will be set to 1.0.
          */
      virtual GaussianDistribution& setParameters( double mu,
                                                   double sig );


   private:


         /// Mean
      double mean;


         /// Standard deviation
      double sigma;


         /// Compute internal parameters
      void recompute(void);


         // Internal parameters
      double a, b;

   };  // End of class "GaussianDistribution"

      //@}

}  // End of namespace gpstk
#endif   // GAUSSIANDISTRIBUTION_HPP
