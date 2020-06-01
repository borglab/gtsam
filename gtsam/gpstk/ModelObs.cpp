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
 * @file ModelObs.cpp
 * This is a class to compute modeled (corrected) observations from a mobile
 * receiver using GNSS data structures.
 */

#include <gtsam/gpstk/ModelObs.hpp>


namespace gpstk
{


// Returns a string identifying this object.
std::string ModelObs::getClassName() const
{
        return "ModelObs";
}



/* Explicit constructor, taking as input initial receiver
 * coordinates, default ionospheric and tropospheric models,
 * ephemeris to be used, default observable and whether TGD will
 * be computed or not.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs::ModelObs( const Position& RxCoordinates,
                    IonoModelStore& dIonoModel,
                    TropModel& dTropoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        Prepare(RxCoordinates);
        setDefaultIonoModel(dIonoModel);
        setDefaultTropoModel(dTropoModel);
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input initial receiver
 * coordinates, default ionospheric model, ephemeris to be used,
 * default observable and whether TGD will be computed or not.
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
ModelObs::ModelObs( const Position& RxCoordinates,
                    IonoModelStore& dIonoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        Prepare(RxCoordinates);
        setDefaultIonoModel(dIonoModel);
        pDefaultTropoModel = NULL;
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input initial receiver
 * coordinates, default tropospheric model, ephemeris to be used,
 * default observable and whether TGD will be computed or not.
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
ModelObs::ModelObs( const Position& RxCoordinates,
                    TropModel& dTropoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        Prepare(RxCoordinates);
        pDefaultIonoModel = NULL;
        setDefaultTropoModel(dTropoModel);
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input initial receiver
 * coordinates, ephemeris to be used, default observable and
 * whether TGD will be computed or not.
 *
 * Both the tropospheric and ionospheric models will be set to NULL.
 *
 * @param RxCoordinates Initial receiver coordinates.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs::ModelObs( const Position& RxCoordinates,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        Prepare(RxCoordinates);
        pDefaultIonoModel = NULL;
        pDefaultTropoModel = NULL;
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input default ionospheric and
 * tropospheric models, ephemeris to be used, default observable
 * and whether TGD will be computed or not.
 *
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs::ModelObs( IonoModelStore& dIonoModel,
                    TropModel& dTropoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        setDefaultIonoModel(dIonoModel);
        setDefaultTropoModel(dTropoModel);
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input default ionospheric model,
 * ephemeris to be used, default observable and whether TGD will be
 * computed or not.
 *
 * @param dIonoModel    Ionospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 * @sa DataStructures.hpp.
 */
ModelObs::ModelObs( IonoModelStore& dIonoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        setDefaultIonoModel(dIonoModel);
        pDefaultTropoModel = NULL;
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Explicit constructor, taking as input default tropospheric model,
 * ephemeris to be used, default observable and whether TGD will be
 * computed or not.
 *
 * @param dTropoModel   Tropospheric model to be used by default.
 * @param dObservable   Observable type to be used by default.
 * @param dEphemeris    XvtStore<SatID> object to be used by default.
 * @param usetgd        Whether TGD will be used by default or not.
 *
 */
ModelObs::ModelObs( TropModel& dTropoModel,
                    XvtStore<SatID>& dEphemeris,
                    const TypeID& dObservable,
                    bool usetgd )
{

        InitializeValues();
        pDefaultIonoModel = NULL;
        setDefaultTropoModel(dTropoModel);
        setDefaultObservable(dObservable);
        setDefaultEphemeris(dEphemeris);
        useTGD = usetgd;

}     // End of 'ModelObs::ModelObs()'



/* Method to set an a priori position of receiver using
 * Bancroft's method.
 *
 * @param Tr            Time of observation
 * @param Satellite     std::vector of satellites in view
 * @param Pseudorange   std::vector of pseudoranges measured from mobile
 *                      station to satellites
 * @param Eph           Satellites Ephemeris
 *
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
int ModelObs::Prepare( const CommonTime& Tr,
                       std::vector<SatID>& Satellite,
                       std::vector<double>& Pseudorange,
                       const XvtStore<SatID>& Eph )
{

        Matrix<double> SVP;
        Bancroft Ban;
        Vector<double> vPos;
        PRSolution2 raimObj;

        try
        {
                raimObj.PrepareAutonomousSolution(Tr, Satellite, Pseudorange, Eph, SVP);
                if( Ban.Compute(SVP, vPos) < 0 )
                {
                        return -1;
                }
        }
        catch(...)
        {
                return -1;
        }

        return Prepare(vPos(0), vPos(1), vPos(2));

}     // End of method 'ModelObs::Prepare()'



/* Method to set an a priori position of receiver using
 * Bancroft's method.
 *
 * @param time      CommonTime object for this epoch
 * @param data      A satTypeValueMap data structure holding the data
 *
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
int ModelObs::Prepare( const CommonTime& time,
                       const satTypeValueMap& data )
{

        int i;
        std::vector<SatID> vSat;
        std::vector<double> vPR;
        Vector<SatID> Satellite( data.getVectorOfSatID() );
        Vector<double> Pseudorange(
                data.getVectorOfTypeID( getDefaultObservable() ) );

        // Convert from gpstk::Vector to std::vector
        for(i = 0; i < (int)Satellite.size(); i++)
        {
                vSat.push_back(Satellite[i]);
        }

        for(i = 0; i < (int)Pseudorange.size(); i++)
        {
                vPR.push_back(Pseudorange[i]);
        }

        return Prepare(time, vSat, vPR, (*(getDefaultEphemeris())) );

}     // End of method 'ModelObs::Prepare()'



/* Method to set the initial (a priori) position of receiver before
 * Compute() method.
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
int ModelObs::Prepare( const double& aRx,
                       const double& bRx,
                       const double& cRx,
                       Position::CoordinateSystem s,
                       EllipsoidModel *ell,
                       ReferenceFrame frame )
{

        int result = setInitialRxPosition(aRx, bRx, cRx, s, ell, frame);

        // If everything is OK, the model is prepared
        if( result ==0 )
        {
                modelPrepared = true;
        }
        else
        {
                modelPrepared = false;
        }

        return result;

}     // End of method 'ModelObs::Prepare()'



/* Method to set the initial (a priori) position of receiver before
 * Compute() method.
 * @return
 *  0 if OK
 *  -1 if problems arose
 */
int ModelObs::Prepare(const Position& RxCoordinates)
{

        int result = setInitialRxPosition(RxCoordinates);

        // If everything is OK, the model is prepared
        if( result ==0 )
        {
                modelPrepared = true;
        }
        else
        {
                modelPrepared = false;
        }

        return result;

}     // End of method 'ModelObs::Prepare()'



/* Returns a satTypeValueMap object, adding the new data generated
 * when calling a modeling object.
 *
 * @param time      Epoch.
 * @param gData     Data object holding the data.
 */
satTypeValueMap& ModelObs::Process( const CommonTime& time,
                                    satTypeValueMap& gData )
throw(ProcessingException)
{

        try
        {

                // First, if the model is not prepared let's take care of it
                if( !getModelPrepared() )
                {
                        Prepare(time, gData);
                }

                ModelObsFixedStation::Process(time, gData);

                return gData;

        }
        catch(Exception& u)
        {
                // Throw an exception if something unexpected happens
                ProcessingException e( getClassName() + ":"
                                       + u.what() );

                GPSTK_THROW(e);

        }

}      // End of method 'ModelObs::Process()'


}  // End of namespace gpstk
