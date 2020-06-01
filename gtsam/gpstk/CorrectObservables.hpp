#pragma ident "$Id$"

/**
 * @file CorrectObservables.hpp
 * This class corrects observables from effects such as antenna excentricity,
 * difference in phase centers, offsets due to tide effects, etc.
 */

#ifndef GPSTK_CORRECTOBSERVABLES_HPP
#define GPSTK_CORRECTOBSERVABLES_HPP

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
//  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009
//
//============================================================================



#include <string>
#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/XvtStore.hpp>
#include <gtsam/gpstk/Triple.hpp>
#include <gtsam/gpstk/Position.hpp>
#include <gtsam/gpstk/Antenna.hpp>
#include <gtsam/gpstk/geometry.hpp>



namespace gpstk
{

/** @addtogroup DataStructures */
//@{


/** This class corrects observables from effects such as antenna
 *  excentricity, difference in phase centers, offsets due to
 *  tidal effects, etc.
 *
 * This class is meant to be used with the GNSS data structures objects
 * found in "DataStructures" class.
 *
 * A typical way to use this class follows:
 *
 * @code
 *       // Create the input obs file stream
 *    RinexObsStream rin("ebre0300.02o");
 *
 *       // Loads precise ephemeris object with file data
 *    SP3EphemerisStore SP3EphList;
 *    SP3EphList.loadFile("igs11513.sp3");
 *
 *       // Sets nominal position of receiver
 *    Position nominalPos(4833520.3800, 41536.8300, 4147461.2800);
 *
 *       // Vector from antenna ARP to L1 phase center [UEN](Leica AT504)
 *    Triple offsetL1(0.1093, -0.0003, 0.0003);        // Units in meters
 *
 *       // Vector from antenna ARP to L2 phase center [UEN](Leica AT504)
 *    Triple offsetL2(0.1282, 0.0011, 0.0011);         // Units in meters
 *
 *       // Vector from monument to antenna ARP [UEN] for this station
 *    Triple offsetARP(2.510, 0.300, 1.045);           // Units in meters
 *
 *       // Vector due to tidal effects (previously computed)
 *    Triple tides(0.121, 0.033, -0.016);              // Units in meters
 *
 *
 *   gnssRinex gRin;
 *   CorrectObservables corr( SP3EphList,
 *                            nominalPos,
 *                            offsetL1,
 *                            offsetL2,
 *                            offsetARP,
 *                            tides       );
 *
 *   while(rin >> gRin)
 *   {
 *      gRin >> corr;
 *   }
 * @endcode
 *
 * The "CorrectObservables" object will visit every satellite in the
 * GNSS data structure that is "gRin" and will correct the
 * corresponding observables from the given effects.
 *
 * When used with the ">>" operator, this class returns the same
 * incoming data structure with the observables corrected. Be warned
 * that if a given satellite does not have the observations required,
 * it will be summarily deleted from the data structure.
 *
 */
class CorrectObservables : public ProcessingClass
{
public:

/// Default constructor
CorrectObservables()
        : pEphemeris(NULL), nominalPos(0.0, 0.0, 0.0), useAzimuth(false),
        L1PhaseCenter(0.0, 0.0, 0.0), L2PhaseCenter(0.0, 0.0, 0.0),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 *
 */
CorrectObservables(XvtStore<SatID>& ephem)
        : pEphemeris(&ephem), nominalPos(0.0, 0.0, 0.0), useAzimuth(false),
        L1PhaseCenter(0.0, 0.0, 0.0), L2PhaseCenter(0.0, 0.0, 0.0),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(0.0, 0.0, 0.0), L2PhaseCenter(0.0, 0.0, 0.0),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem         Satellite ephemeris.
 * @param stapos        Nominal position of receiver station.
 * @param antennaObj    Antenna object with information taken from
 *                      Antex file.
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Antenna& antennaObj )
        : pEphemeris(&ephem), nominalPos(stapos), antenna(antennaObj),
        useAzimuth(true),
        L1PhaseCenter(0.0, 0.0, 0.0), L2PhaseCenter(0.0, 0.0, 0.0),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 * @param L1pc      Position of antenna L1 phase center with respect
 *                  to ARP ([UEN]).
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Triple& L1pc )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(L1pc), L2PhaseCenter(0.0, 0.0, 0.0),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 * @param L1pc      Position of antenna L1 phase center with respect
 *                  to ARP ([UEN]).
 * @param L2pc      Position of antenna L2 phase center with respect
 *                  to ARP ([UEN]).
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Triple& L1pc,
                    const Triple& L2pc )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(L1pc), L2PhaseCenter(L2pc),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(0.0, 0.0, 0.0)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 * @param L1pc      Position of antenna L1 phase center with respect
 *                  to ARP ([UEN]).
 * @param L2pc      Position of antenna L2 phase center with respect
 *                  to ARP ([UEN]).
 * @param extra     Extra biases affecting monument, such as tidal
 *                  effects ([UEN]).
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Triple& L1pc,
                    const Triple& L2pc,
                    const Triple& extra )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(L1pc), L2PhaseCenter(L2pc),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(0.0, 0.0, 0.0), extraBiases(extra)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 * @param L1pc      Position of antenna L1 phase center with respect
 *                  to ARP ([UEN]).
 * @param L2pc      Position of antenna L2 phase center with respect
 *                  to ARP ([UEN]).
 * @param monument  Vector from monument to ARP ([UEN]).
 * @param extra     Extra biases affecting monument, such as tidal
 *                  effects ([UEN]).
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Triple& L1pc,
                    const Triple& L2pc,
                    const Triple& monument,
                    const Triple& extra )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(L1pc), L2PhaseCenter(L2pc),
        L5PhaseCenter(0.0, 0.0, 0.0), L6PhaseCenter(0.0, 0.0, 0.0),
        L7PhaseCenter(0.0, 0.0, 0.0), L8PhaseCenter(0.0, 0.0, 0.0),
        monumentVector(monument), extraBiases(extra)
{
        setIndex();
};


/** Common constructor
 *
 * @param ephem     Satellite ephemeris.
 * @param stapos    Nominal position of receiver station.
 * @param L1pc      Position of antenna L1 phase center with respect
 *                  to ARP ([UEN]).
 * @param L2pc      Position of antenna L2 phase center with respect
 *                  to ARP ([UEN]).
 * @param L5pc      Position of antenna L5 phase center with respect
 *                  to ARP ([UEN]).
 * @param L6pc      Position of antenna L6 phase center with respect
 *                  to ARP ([UEN]).
 * @param L7pc      Position of antenna L7 phase center with respect
 *                  to ARP ([UEN]).
 * @param L8pc      Position of antenna L8 phase center with respect
 *                  to ARP ([UEN]).
 * @param monument  Vector from monument to ARP ([UEN]).
 * @param extra     Extra biases affecting monument, such as tidal
 *                  effects ([UEN]).
 *
 */
CorrectObservables( XvtStore<SatID>& ephem,
                    const Position& stapos,
                    const Triple& L1pc,
                    const Triple& L2pc,
                    const Triple& L5pc,
                    const Triple& L6pc,
                    const Triple& L7pc,
                    const Triple& L8pc,
                    const Triple& monument,
                    const Triple& extra )
        : pEphemeris(&ephem), nominalPos(stapos), useAzimuth(false),
        L1PhaseCenter(L1pc), L2PhaseCenter(L2pc),
        L5PhaseCenter(L5pc), L6PhaseCenter(L6pc),
        L7PhaseCenter(L7pc), L8PhaseCenter(L8pc),
        monumentVector(monument), extraBiases(extra)
{
        setIndex();
};


/** Returns a satTypeValueMap object, adding the new data generated
 *  when calling this object.
 *
 * @param time      Epoch corresponding to the data.
 * @param gData     Data object holding the data.
 */
virtual satTypeValueMap& Process( const CommonTime& time,
                                  satTypeValueMap& gData )
throw(ProcessingException);


/** Returns a gnnsSatTypeValue object, adding the new data generated
 *  when calling this object.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
throw(ProcessingException)
{
        Process(gData.header.epoch, gData.body); return gData;
};


/** Returns a gnnsRinex object, adding the new data generated when
 *  calling this object.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssRinex& Process(gnssRinex& gData)
throw(ProcessingException)
{
        Process(gData.header.epoch, gData.body); return gData;
};


/// Returns nominal position of receiver station.
virtual Position getNominalPosition(void) const
{
        return nominalPos;
};


/** Sets nominal position of receiver station.
 *
 * @param stapos    Nominal position of receiver station.
 */
virtual CorrectObservables& setNominalPosition(const Position& stapos)
{
        nominalPos = stapos; return (*this);
};


/** Returns a pointer to the satellite ephemeris object
 *  currently in use.
 */
virtual XvtStore<SatID> *getEphemeris(void) const
{
        return pEphemeris;
};


/** Sets satellite ephemeris object to be used.
 *
 * @param ephem     Satellite ephemeris object.
 */
virtual CorrectObservables& setEphemeris(XvtStore<SatID>& ephem)
{
        pEphemeris = &ephem; return (*this);
};


/** Returns position of antenna L1 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL1pc(void) const
{
        return L1PhaseCenter;
};


/** Sets position of antenna L1 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L1pc    L1 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL1pc(const Triple& L1pc)
{
        L1PhaseCenter = L1pc; return (*this);
};


/** Returns position of antenna L2 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL2pc(void) const
{
        return L2PhaseCenter;
};


/** Sets position of antenna L2 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L2pc    L2 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL2pc(const Triple& L2pc)
{
        L2PhaseCenter = L2pc; return (*this);
};


/** Returns position of antenna L5 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL5pc(void) const
{
        return L5PhaseCenter;
};


/** Sets position of antenna L5 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L5pc    L5 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL5pc(const Triple& L5pc)
{
        L5PhaseCenter = L5pc; return (*this);
};


/** Returns position of antenna L6 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL6pc(void) const
{
        return L6PhaseCenter;
};


/** Sets position of antenna L6 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L6pc    L6 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL6pc(const Triple& L6pc)
{
        L6PhaseCenter = L6pc; return (*this);
};


/** Returns position of antenna L7 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL7pc(void) const
{
        return L7PhaseCenter;
};


/** Sets position of antenna L7 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L7pc    L7 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL7pc(const Triple& L7pc)
{
        L7PhaseCenter = L7pc; return (*this);
};


/** Returns position of antenna L8 phase center with respect
 *  to ARP ([UEN]).
 */
virtual Triple getL8pc(void) const
{
        return L8PhaseCenter;
};


/** Sets position of antenna L8 phase center with respect
 *  to ARP ([UEN]).
 *
 * @param L8pc    L8 phase center with respect to ARP ([UEN]).
 */
virtual CorrectObservables& setL8pc(const Triple& L8pc)
{
        L8PhaseCenter = L8pc; return (*this);
};


/** Returns vector from monument to ARP ([UEN]).
 */
virtual Triple getMonument(void) const
{
        return monumentVector;
};


/** Sets vector from monument to ARP ([UEN]).
 *
 * @param monument   Vector from monument to ARP ([UEN]).
 */
virtual CorrectObservables& setMonument(const Triple& monument)
{
        monumentVector = monument; return (*this);
};


/** Returns extra biases affecting monument, such as tidal
 *  effects ([UEN]).
 */
virtual Triple getExtraBiases(void) const
{
        return extraBiases;
};


/** Sets extra biases affecting monument, such as tidal
 *  effects ([UEN]).
 *
 * @param extra   Extra biases affecting monument, such as tidal
 *                effects ([UEN]).
 */
virtual CorrectObservables& setExtraBiases(const Triple& extra)
{
        extraBiases = extra; return (*this);
};


/// Returns the antenna object being used.
virtual Antenna getAntenna(void) const
{
        return antenna;
};


/** Sets the antenna object to be used.
 *
 * @param antennaObj    Antenna object to be used.
 */
virtual CorrectObservables& setAntenna(const Antenna& antennaObj)
{
        antenna = antennaObj; useAzimuth = true; return (*this);
};


/// Returns whether azimuth-dependent antenna patterns are being used.
/// When an Antenna is set, this parameter is true by default.
virtual bool getUseAzimuth(void) const
{
        return useAzimuth;
};


/** Sets whether azimuth-dependent antenna patterns will be used.
 *
 * @param useAzimuthPattern   Whether azimuth patterns will be used.
 */
virtual CorrectObservables& setUseAzimuth(bool useAzimuthPattern)
{
        useAzimuth = useAzimuthPattern; return (*this);
};


/// Returns an index identifying this object.
virtual int getIndex(void) const;


/// Returns a string identifying this object.
virtual std::string getClassName(void) const;


/// Destructor
virtual ~CorrectObservables() {
};


private:


/// Satellite ephemeris to be used.
XvtStore<SatID> *pEphemeris;


/// Receiver position.
Position nominalPos;


/// Antenna object with information taken from Antex file.
Antenna antenna;


/// Whether azimuth-dependent antenna patterns will be used or not
bool useAzimuth;


/// Position of antenna L1 phase center with respect to ARP ([UEN]).
Triple L1PhaseCenter;


/// Position of antenna L2 phase center with respect to ARP ([UEN]).
Triple L2PhaseCenter;


/// Position of antenna L5 phase center with respect to ARP ([UEN]).
Triple L5PhaseCenter;


/// Position of antenna L6 phase center with respect to ARP ([UEN]).
Triple L6PhaseCenter;


/// Position of antenna L7 phase center with respect to ARP ([UEN]).
Triple L7PhaseCenter;


/// Position of antenna L8 phase center with respect to ARP ([UEN]).
Triple L8PhaseCenter;


/// Vector from monument to ARP ([UEN]).
Triple monumentVector;


/// Extra biases affecting monument, such as tide effects ([UEN]).
Triple extraBiases;


/// Initial index assigned to this class.
static int classIndex;

/// Index belonging to this object.
int index;

/// Sets the index and increment classIndex.
void setIndex(void)
{
        index = classIndex++;
};


};    // End of class 'CorrectObservables'


//@}

}  // End of namespace gpstk

#endif  // GPSTK_CORRECTOBSERVABLES_HPP
