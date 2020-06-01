#pragma ident "$Id$"

/**
 * @file Bancroft.cpp 
 * Use Bancroft method to get an initial guess of GPS receiver's position
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2006/2007
//
//============================================================================
 
#include <cstdlib>              // for std::abs()
#include "Bancroft.hpp"
#include "MiscMath.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"

using namespace std;
using namespace gpstk;

namespace gpstk
{

      /* 'Data'   Matrix of data containing observation data in rows, one
       *          row per observation and complying with this format:
       *                    x y z P
       *          Where x,y,z are satellite coordinates in an ECEF system
       *          and P is pseudorange (corrected as much as possible,
       *          specially from satellite clock errors), all expresed
       *          in meters.
       *
       * 'X'      Vector of position solution, in meters. There may be
       *          another solution, that may be accessed with vector
       *          "SecondSolution" if "ChooseOne" is set to "false".
       *
       * Return values:
       *  0  Ok
       * -1  Not enough good data
       * -2  Singular problem
       */
   int Bancroft::Compute( Matrix<double>& Data,
                          Vector<double>& X )
      throw(Exception)
   {

      try
      {

         int N = Data.rows();
         Matrix<double> B(0,4);     // Working matrix

            // Let's test the input data
         if( testInput )
         {

            double satRadius = 0.0;

               // Check each row of B Matrix
            for( int i=0; i < N; i++ )
            {
                  // If Data(i,3) -> Pseudorange is NOT between the allowed
                  // range, then drop line immediately
               if( !( (Data(i,3) >= minPRange) && (Data(i,3) <= maxPRange) ) )
               {
                  continue;
               }

                  // Let's compute distance between Earth center and
                  // satellite position
               satRadius = RSS(Data(i,0), Data(i,1) , Data(i,2));

                  // If satRadius is NOT between the allowed range, then drop
                  // line immediately
               if( !( (satRadius >= minRadius) && (satRadius <= maxRadius) ) )
               {
                  continue;
               }

                  // If everything is ok so far, then extract the good
                  // data row and add it to working matrix
               MatrixRowSlice<double> goodRow(Data,i);
               B = B && goodRow;              

            }

               // Let's redefine "N" and check if we have enough data rows
               // left in a single step
            if( (N = B.rows()) < 4 )
            {
               return -1;  // We need at least 4 data rows
            }

         }  // End of 'if( testInput )...'
         else
         {
               // No input filtering. Working matrix (B) and
               // input matrix (Data) are equal
            B = Data;
         }


         Matrix<double> BT=transpose(B);
         Matrix<double> BTBI(4,4), M(4,4,0.0);
         Vector<double> aux(4), alpha(N), solution1(4), solution2(4);

            // Temporary storage for BT*B. It will be inverted later
         BTBI = BT * B;

            // Let's try to invert BTB matrix
         try
         {
            BTBI = inverseChol( BTBI ); 
         }
         catch(...)
         {
            return -2;
         }

            // Now, let's compute alpha vector
         for( int i=0; i < N; i++ )
         {
               // First, fill auxiliar vector with corresponding satellite
               // position and pseudorange
            aux(0) = B(i,0);
            aux(1) = B(i,1);
            aux(2) = B(i,2);
            aux(3) = B(i,3);
            alpha(i) = 0.5 * Minkowski(aux, aux);
         }

         Vector<double> tau(N,1.0), BTBIBTtau(4), BTBIBTalpha(4);

         BTBIBTtau = BTBI * BT * tau;
         BTBIBTalpha = BTBI * BT * alpha;

            // Now, let's find the coeficients of the second order-equation
         double a(Minkowski(BTBIBTtau, BTBIBTtau));
         double b(2.0 * (Minkowski(BTBIBTtau, BTBIBTalpha) - 1.0));
         double c(Minkowski(BTBIBTalpha, BTBIBTalpha));

            // Calculate discriminant and exit if negative
         double discriminant = b*b - 4.0 * a * c;
         if (discriminant < 0.0)
         {
            return -2;
         }

            // Find possible DELTA values
         double DELTA1 = ( -b + SQRT(discriminant) ) / ( 2.0 * a );
         double DELTA2 = ( -b - SQRT(discriminant) ) / ( 2.0 * a );

            // We need to define M matrix
         M(0,0) = 1.0;
         M(1,1) = 1.0;
         M(2,2) = 1.0;
         M(3,3) = - 1.0;

            // Find possible position solutions with their implicit radii
         solution1 = M *  BTBI * ( BT * DELTA1 * tau + BT * alpha );
         double radius1(RSS(solution1(0), solution1(1), solution1(2)));

         solution2 = M *  BTBI * ( BT * DELTA2 * tau + BT * alpha );
         double radius2(RSS(solution2(0), solution2(1), solution2(2)));

            // Let's choose the right solution
         if ( ChooseOne )
         {
            if ( ABS(CloseTo-radius1) < ABS(CloseTo-radius2) )
            {
               X = solution1;
            }
            else
            {
               X = solution2;
            }
         }
         else
         {
               // Both solutions will be reported
            X = solution1;
            SecondSolution = solution2;
         }
     
         return 0;

      }  // end of first "try"
      catch(Exception& e)
      {
         GPSTK_RETHROW(e);
      }
   }  // end Bancroft::Compute()


      // Another version of Compute method to allow calling it with
      // const Matrix B.
   int Bancroft::Compute( const Matrix<double>& Data,
                          Vector<double>& X )
      throw(Exception) 
   {
      Matrix<double> Datanoconst(Data);

      return Bancroft::Compute(Datanoconst, X);

   }
 

} // namespace gpstk
