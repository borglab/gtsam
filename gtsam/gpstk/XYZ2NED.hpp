//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011
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

/**
 * @file XYZ2NED.hpp
 * This is a class to change the reference base from ECEF XYZ to topocentric
 * North-East-Down (NED).
 */

#ifndef GPSTK_XYZ2NED_HPP
#define GPSTK_XYZ2NED_HPP

#include "geometry.hpp"                   // DEG_TO_RAD
#include "Matrix.hpp"
#include "Position.hpp"
#include "TypeID.hpp"
#include "ProcessingClass.hpp"


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{

      /** This class changes the reference base from an Earth-Centered,
       *  Earth-Fixed (ECEF) system to a North-East-Down (NED) topocentric
       *  system, centered at the provided reference location.
       *
       * The NED system may be used when comparing the relative accuracy
       * of a given GNSS data processing strategy. This is a "right-handed"
       * system, and be aware that "down" is positive and "up" is negative.
       *
       * A typical way to use this class follows:
       *
       * @code
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Reference position of receiver station
       *   Position nominalPos(4833520.2269, 41537.00768, 4147461.489);
       *
       *      // Some more code and definitions here...
       *
       *      // GDS object
       *   gnssRinex gRin;
       *
       *      // Set model defaults. A typical C1-based modeling is used
       *   ModeledPR modelRef( nominalPos,
       *                       ionoStore,
       *                       mopsTM,
       *                       bceStore,
       *                       TypeID::C1,
       *                       true );
       *
       *      // Let's define a new equation definition to adapt
       *      // solver object to base change
       *   TypeIDSet typeSet;
       *   typeSet.insert(TypeID::dLat);
       *   typeSet.insert(TypeID::dLon);
       *   typeSet.insert(TypeID::dH);
       *   typeSet.insert(TypeID::cdt);
       *   gnssEquationDefinition newEq(TypeID::prefitC, typeSet);
       *
       *      // Declare (and tune) a SolverLMS object
       *   SolverLMS solver;
       *   solver.setDefaultEqDefinition(newEq);
       *
       *      // Declare the base-changing object setting the reference position
       *   XYZ2NED baseChange(nominalPos);
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> modelRef >> baseChange >> solver;
       *   }
       *
       * @endcode
       *
       * The "XYZ2NED" object will visit every satellite in the GNSS data
       * structure that is "gRin" and will apply a rotation matrix to
       * coefficients dx, dy and dz of the design matrix, yielding
       * corresponding dLat, dLon and dH for each satellite.
       *
       * Take notice that the design matrix coefficients dx, dy and dz were
       * computed by the "ModeledPR" object, so that step is mandatory.
       *
       * Also, the "XYZ2NED" class is effective when properly coupled with
       * the "solver" object (be it based on LMS or WMS). In order to get
       * this, you must instruct the "solver" object to get the solution using
       * a geometry/design matrix based on dLat, dLon and dH, instead of the
       * defaults (dx, dy and dz).
       *
       * The later is achieved defining an appropriate "gnssEquationDefinition"
       * object and instructing "solver" to use it as the default equation
       * definition.
       *
       * @sa XYZ2NEU.hpp
       */
   class XYZ2NED : public ProcessingClass
   {
   public:


         /// Default constructor.
      XYZ2NED()
         : refLat(0.0), refLon(0.0)
      { init(); };


         /** Common constructor taking reference point latitude and longitude
          *
          * @param lat       Latitude of the reference point.
          * @param lon       Longitude of the reference point.
          */
      XYZ2NED( const double& lat,
               const double& lon )
      { setLatLon(lat, lon); }


         /** Common constructor taking reference point Position object
          *
          * @param refPos    Reference point Position object.
          */
      XYZ2NED(const Position& refPos);


         /** Method to set the latitude of the reference point, in degrees.
          *
          * @param lat      Latitude of the reference point, in degrees.
          *
          * @warning If parameter 'lat' is outside the +90/-90 degrees range,
          * then latitude will be set to 0 degrees.
          */
      virtual XYZ2NED& setLat(const double& lat);


         /// Method to get the latitude of the reference point, in degrees.
      virtual double getLat(void) const
      { return (refLat*RAD_TO_DEG); };


         /** Method to set the longitude of the reference point, in degrees.
          *
          * @param lon       Longitude of the reference point, in degrees.
          */
      virtual XYZ2NED& setLon(const double& lon);


         /// Method to get the longitude of the reference point, in degrees.
      virtual double getLon(void) const
      { return (refLon*RAD_TO_DEG); };


         /** Method to simultaneously set the latitude and longitude of
          *  the reference point, in degrees.
          *
          * @param lat        Latitude of the reference point, in degrees.
          * @param lon        Longitude of the reference point, in degrees.
          *
          * @warning If parameter 'lat' is outside the +90/-90 degrees range,
          * then latitude will be set to 0 degrees.
          */
      virtual XYZ2NED& setLatLon( const double& lat,
                                  const double& lon );


         /** Returns a reference to a satTypeValueMap object after
          *  converting from a geocentric reference system to a topocentric
          *  reference system.
          *
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process(satTypeValueMap& gData)
         throw(ProcessingException);


         /** Returns a reference to a gnssSatTypeValue object after
          *  converting from a geocentric reference system to a topocentric
          *  reference system.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /** Returns a reference to a gnnsRinex object after converting
          *  from a geocentric reference system to a topocentric reference
          *  system.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor.
      virtual ~XYZ2NED() {};


   private:


         /// Latitude of the reference point (topocentric reference),
         /// in radians.
      double refLat;


         /// Longitude of the reference point (topocentric reference),
         /// in radians.
      double refLon;


         /// Rotation matrix.
      Matrix<double> rotationMatrix;


         /// Set (TypeIDSet) containing the types of data to be converted 
         /// (dx, dy, dz).
      TypeIDSet inputSet;


         /// Set (TypeIDSet) containing the resulting types of data
         /// (dLat, dLon, dH).
      TypeIDSet outputSet;


         /// This method builds the rotation matrix according to 'refLat'
         /// and 'refLon' values.
      virtual void init();


   }; // End of class 'XYZ2NED'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_XYZ2NED_HPP
