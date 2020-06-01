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
 * @file ModelObsFixedStation.hpp
 * This is a class to compute modeled (corrected) observations from a
 * reference station (whose position is known), using GNSS data structures.
 */

#ifndef GPSTK_MODELOBSFIXEDSTATION_HPP
#define GPSTK_MODELOBSFIXEDSTATION_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/EphemerisRange.hpp>
#include <gtsam/gpstk/EngEphemeris.hpp>
#include <gtsam/gpstk/XvtStore.hpp>
#include <gtsam/gpstk/GPSEphemerisStore.hpp>
#include <gtsam/gpstk/TropModel.hpp>
#include <gtsam/gpstk/IonoModelStore.hpp>


namespace gpstk
{

/// @ingroup GPSsolutions
//@{

/** This class computes modeled (corrected) observations from a
 *  reference station (whose position is known), using GNSS data
 *  structures (GDS).
 *
 * A typical way to use this class follows:
 *
 * @code
 *      // Input observation file stream
 *   RinexObsStream rin("ebre0300.02o");
 *      // Reference position of receiver station
 *   Position nominalPos(4833520.2269, 41537.00768, 4147461.489);
 *
 *      // Some more code and definitions here...
 *
 *   gnssRinex gRin;      // GNSS data structure for fixed station data
 *
 *      // Set defaults of models. A typical C1-based modeling is used
 *   ModelObsFixedStation model( nominalPos,
 *                               ionoStore,
 *                               mopsTM,
 *                               bceStore,
 *                               TypeID::C1,
 *                               true );
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
 * The "ModelObsFixedStation" object will visit every satellite in
 * the GNSS data structure that is "gRin" and will try to compute
 * its model: Prefit residual, geometric distance, relativity delay,
 * ionospheric/tropospheric corrections, geometry matrix, etc.
 *
 * When used with the ">>" operator, this class returns the same
 * incoming data structure with the extra data inserted along their
 * corresponding satellites. Be warned that if a given satellite does
 * not have the observations required, it will be summarily deleted
 * from the data structure.
 *
 * @sa ModelObs.hpp for modeling data from a moving receiver.
 *
 */
class ModelObsFixedStation : public ProcessingClass
{
public:

/// Default constructor. Models C1 observations, use TGD,
/// but doesn't apply atmospheric models
ModelObsFixedStation()
        : minElev(10.0), useTGD(true), pDefaultIonoModel(NULL),
        pDefaultTropoModel(NULL), defaultObservable(TypeID::C1),
        pDefaultEphemeris(NULL)
{
        InitializeValues();
};


/** Explicit constructor taking as input reference
 *  station coordinates.
 *
 * Those coordinates may be Cartesian (X, Y, Z in meters) or Geodetic
 * (Latitude, Longitude, Altitude), but defaults to Cartesian.
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
ModelObsFixedStation( const double& aRx,
                      const double& bRx,
                      const double& cRx,
                      Position::CoordinateSystem s = Position::Cartesian,
                      EllipsoidModel *ell = NULL,
                      ReferenceFrame frame = ReferenceFrame::Unknown );


/// Explicit constructor, taking as input a Position object
/// containing reference station coordinates.
ModelObsFixedStation(const Position& RxCoordinates);


/** Explicit constructor, taking as input reference station
 *  coordinates, default ionospheric and tropospheric models,
 *  ephemeris to be used, default observable and whether TGD will
 *  be computed or not.
 *
 * @param RxCoordinates Reference station coordinates.
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dEphemeris    EphemerisStore object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObsFixedStation( const Position& RxCoordinates,
                      IonoModelStore& dIonoModel,
                      TropModel& dTropoModel,
                      XvtStore<SatID>& dEphemeris,
                      const TypeID& dObservable,
                      bool usetgd = true );


/** Explicit constructor, taking as input reference station
 *  coordinates, default ionospheric model, ephemeris to be used,
 *  default observable and whether TGD will be computed or not.
 *
 * The default tropospheric model will be set to NULL.
 *
 * @param RxCoordinates Reference station coordinates.
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dEphemeris    EphemerisStore object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObsFixedStation( const Position& RxCoordinates,
                      IonoModelStore& dIonoModel,
                      XvtStore<SatID>& dEphemeris,
                      const TypeID& dObservable,
                      bool usetgd = true );


/** Explicit constructor, taking as input reference station
 *  coordinates, default tropospheric model, ephemeris to be used,
 *  default observable and whether TGD will be computed or not.
 *
 * The default ionospheric model will be set to NULL.
 *
 * @param RxCoordinates Reference station coordinates.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dEphemeris    EphemerisStore object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObsFixedStation( const Position& RxCoordinates,
                      TropModel& dTropoModel,
                      XvtStore<SatID>& dEphemeris,
                      const TypeID& dObservable,
                      bool usetgd = true );


/** Explicit constructor, taking as input reference station
 *  coordinates, ephemeris to be used, default observable and whether
 *  TGD will be computed or not.
 *
 * Both the tropospheric and ionospheric models will be set to NULL.
 *
 * @param RxCoordinates Reference station coordinates.
 * @param dEphemeris    EphemerisStore object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObsFixedStation( const Position& RxCoordinates,
                      XvtStore<SatID>& dEphemeris,
                      const TypeID& dObservable,
                      bool usetgd = true);


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
{
        Process(gData.header.epoch, gData.body); return gData;
};


/** Returns a gnnsRinex object, adding the new data generated when
 *  calling a modeling object.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssRinex& Process(gnssRinex& gData)
throw(ProcessingException)
{
        Process(gData.header.epoch, gData.body); return gData;
};


/// Method to get satellite elevation cut-off angle. By default, it
/// is set to 10 degrees.
virtual double getMinElev() const
{
        return minElev;
};


/// Method to set satellite elevation cut-off angle. By default, it
/// is set to 10 degrees.
virtual ModelObsFixedStation& setMinElev(double newElevation)
{
        minElev = newElevation; return (*this);
};


/// Method to set if instrumental delays (TGD) will be used or not
/// in the modeling (it is set to true by default).
virtual ModelObsFixedStation& setTGD(bool use)
{
        useTGD = use; return (*this);
};


/// Method to get a pointer to the default ionospheric model.
virtual IonoModelStore* getDefaultIonoModel() const
{
        return pDefaultIonoModel;
};


/// Method to set a NULL ionospheric model.
virtual ModelObsFixedStation& setNULLIonoModel()
{
        pDefaultIonoModel = NULL; return (*this);
};


/** Method to set the default ionospheric model.
 * @param dIonoModel    Ionospheric model to be used by default.
 */
virtual ModelObsFixedStation& setDefaultIonoModel(
        IonoModelStore& dIonoModel)
{
        pDefaultIonoModel = &dIonoModel; return (*this);
};


/// Method to get a pointer to the default tropospheric model.
virtual TropModel* getDefaultTropoModel() const
{
        return pDefaultTropoModel;
};


/// Method to set a NULL tropospheric model.
virtual ModelObsFixedStation& setNULLTropoModel()
{
        pDefaultTropoModel = NULL; return (*this);
};


/** Method to set the default tropospheric model.
 * @param dTropoModel    Tropospheric model to be used by default.
 */
virtual ModelObsFixedStation& setDefaultTropoModel(
        TropModel& dTropoModel)
{
        pDefaultTropoModel = &dTropoModel; return (*this);
};


/// Method to get the default observable being used with GNSS
/// data structures.
virtual TypeID getDefaultObservable() const
{
        return defaultObservable;
};


/** Method to set the default observable to be used when fed with
 *  GNSS data structures.
 * @param type      TypeID object to be used by default
 */
virtual ModelObsFixedStation& setDefaultObservable(const TypeID& type)
{
        defaultObservable = type; return (*this);
};


/// Method to get a pointer to the default XvtStore<SatID> to be used
/// with GNSS data structures.
virtual XvtStore<SatID>* getDefaultEphemeris() const
{
        return pDefaultEphemeris;
};


/** Method to set the default XvtStore<SatID> to be used with GNSS
 *  data structures.
 *
 * @param ephem     XvtStore<SatID> object to be used by default
 */
virtual ModelObsFixedStation& setDefaultEphemeris(XvtStore<SatID>& ephem)
{
        pDefaultEphemeris = &ephem; return (*this);
};


/// Either estimated or "a priori" position of receiver
Position rxPos;


/// Returns a string identifying this object.
virtual std::string getClassName(void) const;


/// Destructor.
virtual ~ModelObsFixedStation() {
};


protected:


/** Compute the modeled pseudoranges, given satellite ID's,
 *  pseudoranges and other data.
 *
 * @param Tr            Measured time of reception of the data.
 * @param Satellite     Vector of satellites.
 * @param Pseudorange   Vector of raw pseudoranges (parallel to
 *                      satellite), in meters.
 * @param Eph           EphemerisStore to be used.
 * @param pTropModel    Pointer to tropospheric model to be used.
 *                      By default points to NULL.
 * @param pIonoModel    Pointer to ionospheric model to be used.
 *                      By default points to NULL.
 *
 * @return
 *  Number of satellites with valid data
 *
 * @sa TropModel.hpp, IonoModelStore.hpp.
 */
int Compute( const CommonTime& Tr,
             Vector<SatID>& Satellite,
             Vector<double>& Pseudorange,
             const XvtStore<SatID>& Eph,
             TropModel *pTropModel = NULL,
             IonoModelStore *pIonoModel = NULL )
throw(Exception);


/// The elevation cut-off angle for accepted satellites.
/// By default it is set to 10 degrees.
double minElev;

/// Boolean variable indicating if SV instrumental delays (TGD) will
/// be included  in results. It is true by default.
bool useTGD;

/// Pointer to default ionospheric model.
IonoModelStore *pDefaultIonoModel;

/// Pointer to default tropospheric model.
TropModel *pDefaultTropoModel;

/// Default observable to be used when fed with GNSS data structures.
TypeID defaultObservable;

/// Pointer to default XvtStore<SatID> object when working with GNSS
/// data structures.
XvtStore<SatID>* pDefaultEphemeris;

/// Initialization method
virtual void InitializeValues()
{
        setInitialRxPosition();
};


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


/// Method to get the tropospheric corrections.
virtual double getTropoCorrections( TropModel *pTropModel,
                                    double elevation );


/// Method to get the ionospheric corrections.
virtual double getIonoCorrections( IonoModelStore *pIonoModel,
                                   CommonTime Tr,
                                   Position rxGeo,
                                   double elevation,
                                   double azimuth );


/// Method to get TGD corrections.
virtual double getTGDCorrections( CommonTime Tr,
                                  const XvtStore<SatID>& Eph,
                                  SatID sat );


};    // End of class 'ModelObsFixedStation'

//@}

}  // End of namespace gpstk

#endif   // GPSTK_MODELOBSFIXEDSTATION_HPP
