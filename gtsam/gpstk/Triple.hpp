#pragma ident "$Id$"

/**
 * @file Triple.hpp
 * Three element double vectors, for use with geodetic coordinates
 */

#ifndef GPSTK_TRIPLE_HPP
#define GPSTK_TRIPLE_HPP

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

#include <valarray>
#include <vector>
#include "Exception.hpp"
#include "Vector.hpp"

namespace gpstk
{
    /** @addtogroup geodeticgroup */
    //@{

      /// Thrown when a gpstk::Triple operation can't be completed.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(GeometryException, gpstk::Exception);

      /**
       * Three-dimensional vectors.  This class provides mathematical
       * functions for 3D vectors, including some functions specific
       * to orbital tracking.
       */
   class Triple
   {
   public:
         /// Default constructor, initialize as triple.
      Triple();

         /// Copy constructor.
      Triple(const Triple& right);

         /// Construct from three doubles.
      Triple(double a, 
             double b, 
             double c);

         /// Destructor
      virtual ~Triple() {}

         /// Assignment operator.
      Triple& operator=(const Triple& right);

         /** Assign from valarray.
          * @throw GeometryException if right.size() != 3.
          */
      Triple& operator=(const std::valarray<double>& right)
         throw(GeometryException);

         
         /// Return the data as a Vector<double> object
      Vector<double> toVector();


         /// Return the data as a std::vector object
      std::vector<double> toStdVector();

         /**
          * Computes the Dot Product of two vectors
          * @param right vector to compute dot product with.
          * @return The dot product of \c this and \c right
          */
      double dot(const Triple& right) const 
         throw();
   
         /**
          * Computes the Cross Product of two vectors
          * @param right vector to compute cross product with
          * @return The cross product of \c v1 and \c v2
          */
      Triple cross(const Triple& right) const
         throw();
   
         /**
          * Computes the Magnigude of this vector
          */
      double mag() const 
         throw();
   
         /**
          * Returns the unit vector of this vector
          */
      Triple unitVector() const
      	 throw(GeometryException);
      
         /**
          * Computes the Cosine of the Angle Between this vector and another.
          * @param right the other vector
          * @return The cosine of the angle between \c this and \c right
          */
      double cosVector(const Triple& right) const 
         throw(GeometryException);
      
         /**
          * Computes the slant range between this vector and another
          * @param right A Vector
          * @return The slant range between \c this and \c right
          */
      double slantRange(const Triple& right) const 
         throw();
      
         /**
          * Computes the elevation of a point with respect to this
          * point.
          * @param right The second point
          * @return The elevation of \c right relative to \c this
          */
      double elvAngle(const Triple& right) const 
         throw(GeometryException);
      
         /**
          * Computes an azimuth from this point.
          * @param right The position to determine azimuth of.
          * @return The azimuth of \c right relative to \c this
          */ 
      double azAngle(const Triple& right) const 
         throw(GeometryException);
      
         /** Computes rotation about axis X.
          * @param angle    Angle to rotate, in degrees
          * @return A triple which is the original triple rotated angle about X
          */
      Triple R1(const double& angle) const
         throw();
   
      
         /** Computes rotation about axis Y.
          * @param angle    Angle to rotate, in degrees
          * @return A triple which is the original triple rotated angle about Y
          */
      Triple R2(const double& angle) const
         throw();
   
      
         /** Computes rotation about axis Z.
          * @param angle    Angle to rotate, in degrees
          * @return A triple which is the original triple rotated angle about Z
          */
      Triple R3(const double& angle) const
         throw();
   
         /**
          * Return a reference to the element at /a index.
          * @param index the index of the element to return.
          * @return the reference to the requested element.
          */
      double& operator[](const size_t index)
         { return theArray[index]; }

         /**
          * Return the value of the element at /a index.
          * @param index the index of the element to return.
          * @return the value of the element at /a index.
          */
      double operator[](const size_t index) const
         { return theArray[index]; }

     
         /**
          * Equality Operator.
          * @param right the Triple to test equality against
          * @return true if left is equal to right
        */
      bool operator== (const Triple& right) const ;
     

         /**
          * Difference Operator.
          * @param right the Triple to subtract from this object
          * @return a Triple containing the difference between *this and right
          */
      Triple operator-(const Triple& right) const ;

         /**
          * Sum Operator.
          * @param right the Triple to add to this object
          * @return a Triple containing the sum of *this and right
          */
      Triple operator+(const Triple& right) const ;

         /**
          * Multiplication Operator.
          * @right the scale by which to multiply a Triple
          * @rhs   the Triple to scale 
          * @return a Triple containing the scaled result
          */
      friend Triple operator*(double right, const Triple& rhs);

         /// Return the size of this object.
      size_t size(void) const
         { return theArray.size(); }

         /**
          * Output operator for dvec
          * @param s output stream to which \c v is sent
          * @param v dvec that is sent to \c s
          */
      friend std::ostream& operator<<(std::ostream& s, 
                                      const gpstk::Triple& v);
      
      std::valarray<double> theArray;

   }; // class Triple

   //@}

} // namespace gpstk


#endif
