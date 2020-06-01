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
 * @file ModelObs.hpp
 * This is a class to compute modeled (corrected) observations from a mobile
 * receiver using GNSS data structures.
 */

#ifndef GPSTK_MODELOBS_HPP
#define GPSTK_MODELOBS_HPP

#include <gtsam/gpstk/ModelObsFixedStation.hpp>
#include <gtsam/gpstk/Bancroft.hpp>
#include <gtsam/gpstk/PRSolution2.hpp>

namespace gpstk
{

/// @ingroup GPSsolutions
//@{

/** This class compute modeled (corrected) observations from satellites
 *  to a mobile receiver using GNSS data structures (GDS).
 *
 * The main difference between this class and ModelObsFixedStation is
 * that for a mobile receiver we should "prepare" the computation giving
 * an estimate of the mobile station position. This position may be the
 * last known position or it may be estimated using a method such as
 * Bancroft's.
 *
 * Prepare() method is used for this.
 *
 * A typical way to use this class follows:
 *
 * @code
 *   RinexObsStream rin("ebre0300.02o");  // Data stream
 *   RinexNavStream rnavin("brdc0300.02n");   // Ephemeris data stream
 *   RinexNavData rNavData;
 *   GPSEphemerisStore bceStore;
 *   while (rnavin >> rNavData) bceStore.addEphemeris(rNavData);
 *   bceStore.SearchPast();  // This is the default
 *
 *   RinexNavHeader rNavHeader;
 *   IonoModelStore ionoStore;
 *   IonoModel ioModel;
 *   rnavin >> rNavHeader;    // Read navigation RINEX header
 *   ioModel.setModel(rNavHeader.ionAlpha, rNavHeader.ionBeta);
 *   ionoStore.addIonoModel(CommonTime::BEGINNING_OF_TIME, ioModel);
 *
 *      // EBRE station nominal position
 *   Position nominalPos(4833520.3800, 41536.8300, 4147461.2800);
 *
 *      // Declare a tropospheric model object, setting the defaults
 *   MOPSTropModel mopsTM( nominalPos.getAltitude(),
 *                         nominalPos.getGeodeticLatitude(),
 *                         30 );
 *
 *      // Declare the modeler object, setting all the parameters
 *      // in one pass
 *      // As stated, it will compute the model using the C1 observable
 *   ModelObs model( ionoStore,
 *                   mopsTM,
 *                   bceStore,
 *                   TypeID::C1 );
 *
 *      // Set the initial apriori position using the Bancroft method
 *   model.Prepare();
 *
 *   gnssRinex gRin;
 *
 *   while(rin >> gRin)
 *   {
 *      gRin >> model;
 *   }
 * @endcode
 *
 * The "ModelObs" object will visit every satellite in the
 * GNSS data structure that is "gRin" and will try to compute its
 * model: Prefit residual, geometric distance, relativity delay,
 * ionospheric/tropospheric corrections, geometry matrix, etc.
 *
 * When used with the ">>" operator, this class returns the same
 * incoming data structure with the extra data inserted along their
 * corresponding satellites. Be warned that if a given satellite does
 * not have the observations required, it will be summarily deleted
 * from the data structure.
 *
 * @sa ModelObsFixedStation.hpp for modeling data for a fixed station
 * whose coordinates are known.
 *
 */
class ModelObs : public ModelObsFixedStation
{
public:

/// Implicit constructor
ModelObs() : modelPrepared(false)
{
};


/** Explicit constructor, taking as input initial receiver
 *  coordinates, default ionospheric and tropospheric models,
 *  ephemeris to be used, default observable and whether TGD will
 *  be computed or not.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( const Position& RxCoordinates,
          IonoModelStore& dIonoModel,
          TropModel& dTropoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input initial receiver
 *  coordinates, default ionospheric model, ephemeris to be used,
 *  default observable and whether TGD will be computed or not.
 *
 * The default tropospheric model will be set to NULL.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( const Position& RxCoordinates,
          IonoModelStore& dIonoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input initial receiver
 *  coordinates, default tropospheric model, ephemeris to be used,
 *  default observable and whether TGD will be computed or not.
 *
 * The default ionospheric model will be set to NULL.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( const Position& RxCoordinates,
          TropModel& dTropoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input initial receiver
 *  coordinates, ephemeris to be used, default observable and
 *  whether TGD will be computed or not.
 *
 * Both the tropospheric and ionospheric models will be set to NULL.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( const Position& RxCoordinates,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input default ionospheric and
 *  tropospheric models, ephemeris to be used, default observable
 *  and whether TGD will be computed or not.
 *
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( IonoModelStore& dIonoModel,
          TropModel& dTropoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input default ionospheric model,
 *  ephemeris to be used, default observable and whether TGD will be
 *  computed or not.
 *
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 * @sa DataStructures.hpp.
 */
ModelObs( IonoModelStore& dIonoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Explicit constructor, taking as input default tropospheric model,
 *  ephemeris to be used, default observable and whether TGD will be
 *  computed or not.
 *
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs( TropModel& dTropoModel,
          XvtStore<SatID>& dEphemeris,
          const TypeID& dObservable,
          bool usetgd = true );


/** Method to set an a priori position of receiver using
 *  Bancroft's method.
 *
 * @param Tr            Time of observation
 * @param Satellite     std::vector of satellites in view
 * @param Pseudorange   std::vector of pseudoranges measured from
 *                      mobile station to satellites
 * @param Eph           Satellites Ephemeris
 *
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
virtual int Prepare( const CommonTime& Tr,
                     std::vector<SatID>& Satellite,
                     std::vector<double>& Pseudorange,
                     const XvtStore<SatID>& Eph );


/** Method to set an a priori position of receiver using
 *  Bancroft's method.
 *
 * @param time      CommonTime object for this epoch
 * @param data      A satTypeValueMap data structure holding the data
 *
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
virtual int Prepare( const CommonTime& time,
                     const satTypeValueMap& data );


/** Method to set an a priori position of receiver using
 *  Bancroft's method.
 *
 * @param gData         GNSS data structure to be used
 *
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
virtual int Prepare(const gnssSatTypeValue& gData)
{
        return (Prepare(gData.header.epoch, gData.body));
};


/** Method to set the initial (a priori) position of receiver before
 *  Compute() method.
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
virtual int Prepare( const double& aRx,
                     const double& bRx,
                     const double& cRx,
                     Position::CoordinateSystem s = Position::Cartesian,
                     EllipsoidModel *ell = NULL,
                     ReferenceFrame frame = ReferenceFrame::Unknown );


/** Method to set the initial (a priori) position of receiver before
 *  Compute() method.
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
virtual int Prepare(const Position& RxCoordinates);


/** Returns a satTypeValueMap object, adding the new data generated
 *  when calling a modeling object.
 *
 * @param time      Epoch.
 * @param gData     Data object holding the data.
 */
virtual satTypeValueMap& Process( const CommonTime& time,
                                  satTypeValueMap& gData )
throw(ProcessingException);


/// Method to get if the model has been prepared.
inline bool getModelPrepared() const
{
        return modelPrepared;
};


/** Method to forcefully set whether the model has been prepared.
 *
 * @param prepare       Boolean indicating whether the model has
 *                      been prepared.
 */
inline ModelObs& setModelPrepared(const bool& prepare)
{
        modelPrepared = prepare; return (*this);
};


/// Returns a string identifying this object.
virtual std::string getClassName(void) const;


/// Destructor.
virtual ~ModelObs() {
};


protected:


bool modelPrepared;


};    // End of class 'ModelObs'

//@}

}  // End of namespace gpstk

#endif   // GPSTK_MODELOBS_HPP
