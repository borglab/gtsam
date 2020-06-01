#pragma ident "$Id$"

/**
 * @file Bancroft.hpp 
 * Use Bancroft method to get an initial guess of GPS receiver's position
 */
 
#ifndef BANCROFT_HPP
#define BANCROFT_HPP

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

#include <vector>
#include "Matrix.hpp"

namespace gpstk
{
      /** @defgroup GPSsolutions GPS solution algorithms and Tropospheric
       *  models
       */

      //@{
       	
      /** This class defines an algebraic algorithm to get an initial guess of
       *  GPS receiver's position given satellites' positions and pseudoranges.
       *
       * The algorithm is based on Bancroft's Method as presented in: Yang,
       * Ming & Kuo-Hwa Chen. ''Performance Assessment of a Noniterative
       * Algorithm for Global Positioning System (GPS) Absolute
       * Positioning''. Proc. Natl. Sci. Counc. ROC(A). Vol. 25, No. 2,
       * 2001. pp. 102-106.
       */
   class Bancroft
   {
   public:

         /// Constructor
      Bancroft()
         throw(Exception) : SecondSolution(4,0.0)
      { 
         testInput= true;
         ChooseOne = true;
         CloseTo = 6378137.0;
         minPRange = 15000000.0;
         maxPRange = 30000000.0;
         minRadius = 23000000.0;
         maxRadius = 29000000.0;
      };


         /** Compute an initial guess of GPS receiver's position , given
          *  satellites' positions and pseudoranges.
          *
          * @param Data    Matrix of data containing observation data in rows,
          *                one row per observation and complying with the
          *                following format:
          *
          *                        x y z P
          *
          *                Where x,y,z are satellite coordinates in an ECEF
          *                system and P is pseudorange (corrected as much as
          *                possible, specially from satellite clock errors),
          *                all expresed in meters.
          *
          * @param X      Vector of position solution, in meters. There may be
          *               another solution that may be accessed with vector
          *               "SecondSolution" if "ChooseOne" is set to "false".
          *
          * @return
          *    0  Ok, 
          *   -1  Not enough good data
          *   -2  Singular problem
          */
      int Compute( Matrix<double>& Data,
                   Vector<double>& X )
         throw(Exception);


         /** Another version of Compute method allowing calls with Matrix B
          *  being const.
          */
      int Compute( const Matrix<double>& Data,
                   Vector<double>& X )
         throw(Exception);


         /** If true, the solution closest to CloseTo criterion will be chosen.
          *  If false, the two posible solutions will be provided.
          */
      bool ChooseOne;


         /** Criterion to decide which solution to choose. The algorithm will
          *  choose the solution closer to this value. By default, it is set
          *  to earth radius, in meters.
          */
      double CloseTo;


         /** If true (the default), the B input Matrix will be screened to get
          *  out suspicious data.
          *
          * It works with minPRange, maxPRange, minRadius and maxRadius to
          * pick up a set of "clean data". However, don't be too picky with
          * these parameters in order to leave room for different GNSS systems
          * and configurations. Anyway, Bancroft will give you just an
          * approximate position.
          */
      bool testInput;


         /// Minimum pseudorange value allowed for input data (in meters).
      double minPRange;


         /// Maximum pseudorange value allowed for input data (in meters).
      double maxPRange;


         /// Minimum allowed distance between Earth center and satellite
         /// position for input data (in meters).
      double minRadius;


         /// Maximum allowed distance between Earth center and satellite
         /// position for input data (in meters).
      double maxRadius;


         /** Vector<double> containing the estimated second position solution
          * (ECEF, meters), if ChooseOne is set to "false".
          */
      Vector<double> SecondSolution;


         /// Destructor.
      virtual ~Bancroft() throw() {};


   }; // end class Bancroft

      //@}

} // namespace gpstk

#endif   // BANCROFT_HPP
