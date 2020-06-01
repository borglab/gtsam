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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011
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
 * @file BasicModel.hpp
 * This is a class to compute the basic parts of a GNSS model, i.e.:
 * Geometric distance, relativity correction, satellite position and
 * velocity at transmission time, satellite elevation and azimuth, etc.
 */

#ifndef GPSTK_BASICMODEL_HPP
#define GPSTK_BASICMODEL_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/EphemerisRange.hpp>
#include <gtsam/gpstk/EngEphemeris.hpp>
#include <gtsam/gpstk/XvtStore.hpp>
#include <gtsam/gpstk/GPSEphemerisStore.hpp>

namespace gpstk
{
      /** @addtogroup GPSsolutions */
      //@{

      /** This is a class to compute the basic parts of a GNSS model, like
       *  geometric distance, relativity correction, satellite position and
       *  velocity at transmission time, satellite elevation and azimuth, etc.
       *
       * This class is intended to be used with GNSS Data Structures (GDS).
       * It is a more modular alternative to classes such as ModelObs
       * and ModelObsFixedStation.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Input observation file stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Load the precise ephemeris file
       *   SP3EphemerisStore sp3Eph;
       *   sp3Eph.loadFile("igs11513.sp3");
       *
       *      // Reference position of receiver station
       *   Position nominalPos(4833520.2269, 41537.00768, 4147461.489);
       *
       *      // Some more code and definitions here...
       *
       *   gnssRinex gRin;  // GNSS data structure for fixed station data
       *
       *      // Set defaults of models. A typical C1-based modeling is used
       *   BasicModel model( nominalPos, sp3Eph );
       *
       *   while(rin >> gRin)
       *   {
       *
       *         // Apply the model on the GDS
       *      gRin >> model;
       *   }
       *
       * @endcode
       *
       * The "BasicModel" object will visit every satellite in
       * the GNSS data structure that is "gRin" and will try to compute
       * its model: Geometric distance, relativity delay, satellite position
       * at transmission time, satellite elevation and azimuth, etc.
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the extra data inserted along their
       * corresponding satellites. Be warned that if a given satellite does
       * not have ephemeris information, it will be summarily deleted
       * from the data structure.
       *
       * @sa ModelObs.hpp and ModelObsFixedStation.hpp for classes carrying
       * out a more complete model.
       *
       */
   class BasicModel : public ProcessingClass
   {
   public:

         /// Default constructor. Observable C1 will be used for computations
         /// and satellites with elevation less than 10 degrees will be
         /// deleted.
      BasicModel()
         : minElev(10.0), pDefaultEphemeris(NULL),
           defaultObservable(TypeID::C1), useTGD(false)
      { setInitialRxPosition(); };


         /** Explicit constructor taking as input reference
          *  station coordinates.
          *
          * Those coordinates may be Cartesian (X, Y, Z in meters) or Geodetic
          * (Latitude, Longitude, Altitude), but defaults to Cartesian.
          *
          * Also, a pointer to GeoidModel may be specified, but default is
          * NULL (in which case WGS84 values will be used).
          *
          * @param aRx   first coordinate [ X(m), or latitude (degrees N) ]
          * @param bRx   second coordinate [ Y(m), or longitude (degrees E) ]
          * @param cRx   third coordinate [ Z, height above ellipsoid or
          *              radius, in meters ]
          * @param s     coordinate system (default is Cartesian, may be set
          *              to Geodetic).
          * @param ell   pointer to EllipsoidModel.
          * @param frame Reference frame associated with this position.
          */
      BasicModel( const double& aRx,
                  const double& bRx,
                  const double& cRx,
                  Position::CoordinateSystem s = Position::Cartesian,
                  EllipsoidModel *ell = NULL,
                  ReferenceFrame frame = ReferenceFrame::Unknown );


         /// Explicit constructor, taking as input a Position object
         /// containing reference station coordinates.
      BasicModel(const Position& RxCoordinates);


         /** Explicit constructor, taking as input reference station
          *  coordinates, ephemeris to be used and whether TGD will
          *  be computed or not.
          *
          * @param RxCoordinates Reference station coordinates.
          * @param dEphemeris    EphemerisStore object to be used by default.
          * @param dObservable   Observable type to be used by default.
          * @param applyTGD      Whether or not C1 observable will be
          *                      corrected from TGD effect.
          *
          */
      BasicModel( const Position& RxCoordinates,
                  XvtStore<SatID>& dEphemeris,
                  const TypeID& dObservable = TypeID::C1,
                  const bool& applyTGD = false );


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling a modeling object.
          *
          * @param time      Epoch.
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process( const CommonTime& time,
                                        satTypeValueMap& gData )
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling a modeling object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling a modeling object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /// Method to get satellite elevation cut-off angle. By default, it
         /// is set to 10 degrees.
      virtual double getMinElev() const
      { return minElev; };


         /// Method to set satellite elevation cut-off angle. By default, it
         /// is set to 10 degrees.
      virtual BasicModel& setMinElev(double newElevation)
      { minElev = newElevation; return (*this); };


         /// Method to get the default observable for computations.
      virtual TypeID getDefaultObservable() const
      { return defaultObservable; };


         /** Method to set the default observable for computations.
          *
          * @param type      TypeID object to be used by default
          */
      virtual BasicModel& setDefaultObservable(const TypeID& type)
      { defaultObservable = type; return (*this); };


         /// Method to get a pointer to the default XvtStore<SatID> to be used
         /// with GNSS data structures.
      virtual XvtStore<SatID>* getDefaultEphemeris() const
      { return pDefaultEphemeris; };


         /** Method to set the default XvtStore<SatID> to be used with GNSS
          *  data structures.
          *
          * @param ephem     XvtStore<SatID> object to be used by default
          */
      virtual BasicModel& setDefaultEphemeris(XvtStore<SatID>& ephem)
      { pDefaultEphemeris = &ephem; return (*this); };


         /// Either estimated or "a priori" position of receiver
      Position rxPos;


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor.
      virtual ~BasicModel() {};


   protected:


         /// The elevation cut-off angle for accepted satellites.
         /// By default it is set to 10 degrees.
      double minElev;


         /// Pointer to default XvtStore<SatID> object when working with GNSS
         /// data structures.
      XvtStore<SatID>* pDefaultEphemeris;


         /// Default observable to be used when fed with GNSS data structures.
      TypeID defaultObservable;


         /// Whether the TGD effect will be applied to C1 observable or not.
      bool useTGD;


         /** Method to set the initial (a priori) position of receiver.
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int setInitialRxPosition( const double& aRx,
                                        const double& bRx,
                                        const double& cRx,
                           Position::CoordinateSystem s = Position::Cartesian,
                                        EllipsoidModel *ell = NULL,
                           ReferenceFrame frame = ReferenceFrame::Unknown );


         /// Method to set the initial (a priori) position of receiver.
      virtual int setInitialRxPosition(const Position& RxCoordinates);


         /// Method to set the initial (a priori) position of receiver.
      virtual int setInitialRxPosition();


         /// Method to get TGD corrections.
      virtual double getTGDCorrections( CommonTime Tr,
                                        const XvtStore<SatID>& Eph,
                                        SatID sat )
         throw();


   }; // End of class 'BasicModel'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_BASICMODEL_HPP
