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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009
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
 * @file TypeID.cpp
 * gpstk::TypeID - Identifies types of values
 */

#include "TypeID.hpp"


namespace gpstk
{

std::map< TypeID::ValueType, std::string > TypeID::tStrings;


TypeID::Initializer TypeIDsingleton;

// It should be initialize by false, NEVER CHANGE IT!!!
bool TypeID::bUserTypeIDRegistered = false;

// Map holding user defined TypeIDs by a string
std::map<std::string,TypeID> TypeID::mapUserTypeID;


// Let's assign type descriptions
TypeID::Initializer::Initializer()
{
        tStrings[Unknown]    = "UnknownType";
        tStrings[C1]         = "C1";
        tStrings[C2]         = "C2";
        tStrings[P1]         = "P1";
        tStrings[P2]         = "P2";
        tStrings[L1]         = "L1";
        tStrings[L2]         = "L2";
        tStrings[D1]         = "D1";
        tStrings[D2]         = "D2";
        tStrings[S1]         = "S1";
        tStrings[S2]         = "S2";
        tStrings[T1]         = "T1";
        tStrings[T2]         = "T2";
        tStrings[SSI1]       = "SSI1";
        tStrings[LLI1]       = "LLI1";
        tStrings[SSI2]       = "SSI2";
        tStrings[LLI2]       = "LLI2";
        tStrings[C5]         = "C5";
        tStrings[L5]         = "L5";
        tStrings[D5]         = "D5";
        tStrings[S5]         = "S5";
        tStrings[SSI5]       = "SSI5";
        tStrings[LLI5]       = "LLI5";
        tStrings[C6]         = "C6";
        tStrings[L6]         = "L6";
        tStrings[D6]         = "D6";
        tStrings[S6]         = "S6";
        tStrings[SSI6]       = "SSI6";
        tStrings[LLI6]       = "LLI6";
        tStrings[C7]         = "C7";
        tStrings[L7]         = "L7";
        tStrings[D7]         = "D7";
        tStrings[S7]         = "S7";
        tStrings[SSI7]       = "SSI7";
        tStrings[LLI7]       = "LLI7";
        tStrings[C8]         = "C8";
        tStrings[L8]         = "L8";
        tStrings[D8]         = "D8";
        tStrings[S8]         = "S8";
        tStrings[SSI8]       = "SSI8";
        tStrings[LLI8]       = "LLI8";
        tStrings[PC]         = "PC";
        tStrings[LC]         = "LC";
        tStrings[PI]         = "PI";
        tStrings[LI]         = "LI";
        tStrings[Pdelta]     = "Pdelta";
        tStrings[Ldelta]     = "Ldelta";
        tStrings[MWubbena]   = "MWubbena";
        tStrings[GRAPHIC1]   = "GRAPHIC1";
        tStrings[GRAPHIC2]   = "GRAPHIC2";
        tStrings[GRAPHIC5]   = "GRAPHIC5";
        tStrings[GRAPHIC6]   = "GRAPHIC6";
        tStrings[GRAPHIC7]   = "GRAPHIC7";
        tStrings[GRAPHIC8]   = "GRAPHIC8";
        tStrings[WL]         = "WL";
        tStrings[WL1]        = "WL1";
        tStrings[WL2]        = "WL2";
        tStrings[WL3]        = "WL3";
        tStrings[WL4]        = "WL4";
        tStrings[EWL]        = "EWL";
        tStrings[L1dot]      = "L1dot";
        tStrings[L1dot2]     = "L1dot2";
        tStrings[L2dot]      = "L2dot";
        tStrings[L2dot2]     = "L2dot2";
        tStrings[L5dot]      = "L5dot";
        tStrings[L5dot2]     = "L5dot2";
        tStrings[P1dot]      = "P1dot";
        tStrings[P1dot2]     = "P1dot2";
        tStrings[P2dot]      = "P2dot";
        tStrings[P2dot2]     = "P2dot2";
        tStrings[P5dot]      = "P5dot";
        tStrings[P5dot2]     = "P5dot2";
        tStrings[L6dot]      = "L6dot";
        tStrings[L6dot2]     = "L6dot2";
        tStrings[L7dot]      = "L7dot";
        tStrings[L7dot2]     = "L7dot2";
        tStrings[L8dot]      = "L8dot";
        tStrings[L8dot2]     = "L8dot2";
        tStrings[LCdot]      = "LCdot";
        tStrings[LCdot2]     = "LCdot2";
        tStrings[LIdot]      = "LIdot";
        tStrings[LIdot2]     = "LIdot2";
        tStrings[Ldeltadot]  = "Ldeltadot";
        tStrings[Ldeltadot2] = "Ldeltadot2";
        tStrings[transmit]   = "transmit";
        tStrings[rho]        = "rho";
        tStrings[rhodot]     = "rhodot";
        tStrings[rhodot2]    = "rhodot2";
        tStrings[dtSat]      = "dtSat";
        tStrings[dtSatdot]   = "dtSatdot";
        tStrings[dtSatdot2]  = "dtSatdot2";
        tStrings[rel]        = "rel";
        tStrings[gravDelay]  = "gravDelay";
        tStrings[tropo]      = "tropo";
        tStrings[dryTropo]   = "dryTropo";
        tStrings[dryMap]     = "dryTropoMap";
        tStrings[wetTropo]   = "wetTropo";
        tStrings[wetMap]     = "wetTropoMap";
        tStrings[tropoSlant] = "slantTropo";
        tStrings[iono]       = "verticalIono";
        tStrings[ionoTEC]    = "TotalElectronContent";
        tStrings[ionoMap]    = "ionoMap";
        tStrings[ionoMap2]   = "ionoMap2";
        tStrings[ionoL1]     = "slantIonoL1";
        tStrings[ionoL2]     = "slantIonoL2";
        tStrings[ionoL5]     = "slantIonoL5";
        tStrings[ionoL6]     = "slantIonoL6";
        tStrings[ionoL7]     = "slantIonoL7";
        tStrings[ionoL8]     = "slantIonoL8";
        tStrings[windUp]     = "windup";
        tStrings[satPCenter] = "satPhaseCenter";
        tStrings[satX]       = "satX";
        tStrings[satY]       = "satY";
        tStrings[satZ]       = "satZ";
        tStrings[satVX]      = "satVX";
        tStrings[satVY]      = "satVY";
        tStrings[satVZ]      = "satVZ";
        tStrings[satAX]      = "satAX";
        tStrings[satAY]      = "satAY";
        tStrings[satAZ]      = "satAZ";
        tStrings[satJ2kX]    = "satJ2kX";
        tStrings[satJ2kY]    = "satJ2kY";
        tStrings[satJ2kZ]    = "satJ2kZ";
        tStrings[satJ2kVX]   = "satJ2kVX";
        tStrings[satJ2kVY]   = "satJ2kVY";
        tStrings[satJ2kVZ]   = "satJ2kVZ";
        tStrings[satJ2kAX]   = "satJ2kAX";
        tStrings[satJ2kAY]   = "satJ2kAY";
        tStrings[satJ2kAZ]   = "satJ2kAZ";
        tStrings[elevation]  = "elevation";
        tStrings[azimuth]    = "azimuth";

        tStrings[CSL1]       = "CSL1";
        tStrings[CSL2]       = "CSL2";
        tStrings[CSL5]       = "CSL5";
        tStrings[CSL6]       = "CSL6";
        tStrings[CSL7]       = "CSL7";
        tStrings[CSL8]       = "CSL8";
        tStrings[satArc]     = "satArc";
        tStrings[BL1]        = "ambiguityL1";
        tStrings[BL2]        = "ambiguityL2";
        tStrings[BL5]        = "ambiguityL5";
        tStrings[BL6]        = "ambiguityL6";
        tStrings[BL7]        = "ambiguityL7";
        tStrings[BL8]        = "ambiguityL8";
        tStrings[BLC]        = "ambiguityLC";
        tStrings[BWL]        = "ambiguityWL";
        tStrings[BWL2]       = "ambiguityWL2";
        tStrings[BWL3]       = "ambiguityWL3";
        tStrings[BWL4]       = "ambiguityWL4";
        tStrings[mpC1]       = "multipathC1";
        tStrings[mpC2]       = "multipathC2";
        tStrings[mpC5]       = "multipathC5";
        tStrings[mpC6]       = "multipathC6";
        tStrings[mpC7]       = "multipathC7";
        tStrings[mpC8]       = "multipathC8";
        tStrings[mpL1]       = "multipathL1";
        tStrings[mpL2]       = "multipathL2";
        tStrings[mpL5]       = "multipathL5";
        tStrings[mpL6]       = "multipathL6";
        tStrings[mpL7]       = "multipathL7";
        tStrings[mpL8]       = "multipathL8";
        tStrings[instC1]     = "instrumentalC1";
        tStrings[instC2]     = "instrumentalC2";
        tStrings[instC5]     = "instrumentalC5";
        tStrings[instC6]     = "instrumentalC6";
        tStrings[instC7]     = "instrumentalC7";
        tStrings[instC8]     = "instrumentalC8";
        tStrings[instL1]     = "instrumentalL1";
        tStrings[instL2]     = "instrumentalL2";
        tStrings[instL5]     = "instrumentalL5";
        tStrings[instL6]     = "instrumentalL6";
        tStrings[instL7]     = "instrumentalL7";
        tStrings[instL8]     = "instrumentalL8";

        tStrings[prefitP1]   = "prefitResidualCodeP1";
        tStrings[prefitP2]   = "prefitResidualCodeP2";
        tStrings[prefitL1]   = "prefitResidualPhaseL1";
        tStrings[prefitL2]   = "prefitResidualPhaseL2";
        tStrings[postfitP1]  = "postfitResidualCodeP1";
        tStrings[postfitP2]  = "postfitResidualCodeP2";
        tStrings[postfitL1]  = "postfitResidualPhaseL1";
        tStrings[postfitL2]  = "postfitResidualPhaseL2";

        tStrings[prefitC5]   = "prefitResidualCodeC5";
        tStrings[prefitL5]   = "prefitResidualPhaseL5";
        tStrings[postfitC5]  = "postfitResidualCodeC5";
        tStrings[postfitL5]  = "postfitResidualPhaseL5";

        tStrings[prefitGRAPHIC1]  = "prefitResidualGRAPHIC1";
        tStrings[prefitGRAPHIC2]  = "prefitResidualGRAPHIC2";
        tStrings[postfitGRAPHIC1] = "postfitResidualGRAPHIC1";
        tStrings[postfitGRAPHIC2] = "postfitResidualGRAPHIC2";
        tStrings[prefitMWubbena]   = "prefitMWubbena";
        tStrings[prefitWL]   = "prefitResidualWL";
        tStrings[prefitWL2]  = "prefitResidualWL2";
        tStrings[prefitWL3]  = "prefitResidualWL3";
        tStrings[prefitWL4]  = "prefitResidualWL4";
        tStrings[postfitWL]  = "postfitResidualWL";
        tStrings[postfitWL2] = "postfitResidualWL2";
        tStrings[postfitWL3] = "postfitResidualWL3";
        tStrings[postfitWL4] = "postfitResidualWL4";
        tStrings[prefitC]    = "prefitResidualCode";
        tStrings[prefitL]    = "prefitResidualPhase";
        tStrings[postfitC]   = "posfitResidualCode";
        tStrings[postfitL]   = "posfitResidualPhase";
        tStrings[dx]         = "dx";
        tStrings[dy]         = "dy";
        tStrings[dz]         = "dz";
        tStrings[cdt]        = "cdt";
        tStrings[cdtSat]     = "cdtSat";
        tStrings[dLat]       = "dLat";
        tStrings[dLon]       = "dLon";
        tStrings[dH]         = "dH";
        tStrings[dSatX]      = "dSatX";
        tStrings[dSatY]      = "dSatY";
        tStrings[dSatZ]      = "dSatZ";
        tStrings[dSatR]      = "dSatR";
        tStrings[dSatT]      = "dSatT";
        tStrings[dSatN]      = "dSatN";
        tStrings[weight]     = "weight";
        tStrings[codeBias]   = "codeBias";

        tStrings[cdtC1]    = "cdtC1";
        tStrings[cdtP1]    = "cdtP1";
        tStrings[cdtC2]    = "cdtC2";
        tStrings[cdtP2]    = "cdtP2";
        tStrings[cdtC5]    = "cdtC5";
        tStrings[cdtP5]    = "cdtP5";
        tStrings[cdtL1]    = "cdtL1";
        tStrings[cdtL2]    = "cdtL2";
        tStrings[cdtL5]    = "cdtL5";
        tStrings[cdtPC]    = "cdtPC";
        tStrings[cdtLC]    = "cdtLC";
        tStrings[cdtWL]    = "cdtWL";
        tStrings[cdtWL2]    = "cdtWL2";
        tStrings[cdtWL3]    = "cdtWL3";
        tStrings[cdtWL4]    = "cdtWL4";
        tStrings[cdtMW]    = "cdtMW";

        tStrings[cdtSatC1]    = "cdtSatC1";
        tStrings[cdtSatP1]    = "cdtSatP1";
        tStrings[cdtSatC2]    = "cdtSatC2";
        tStrings[cdtSatP2]    = "cdtSatP2";
        tStrings[cdtSatC5]    = "cdtSatC5";
        tStrings[cdtSatP5]    = "cdtSatP5";
        tStrings[cdtSatL1]    = "cdtSatL1";
        tStrings[cdtSatL2]    = "cdtSatL2";
        tStrings[cdtSatL5]    = "cdtSatL5";
        tStrings[cdtSatPC]    = "cdtSatPC";
        tStrings[cdtSatLC]    = "cdtSatLC";
        tStrings[cdtSatWL]    = "cdtSatWL";
        tStrings[cdtSatMW]    = "cdtSatMW";

        tStrings[recX]       = "RxPositionX";
        tStrings[recY]       = "RxPositionY";
        tStrings[recZ]       = "RxPositionZ";
        tStrings[recVX]      = "RxVelocityX";
        tStrings[recVY]      = "RxVelocityY";
        tStrings[recVZ]      = "RxVelocityZ";
        tStrings[recAX]      = "RxAccelerationX";
        tStrings[recAY]      = "RxAccelerationY";
        tStrings[recAZ]      = "RxAccelerationZ";
        tStrings[recLat]     = "RxLat";
        tStrings[recLon]     = "RxLon";
        tStrings[recH]       = "RxH";
        tStrings[recVLat]    = "RxVelocityLat";
        tStrings[recVLon]    = "RxVelocityLon";
        tStrings[recVH]      = "RxVelocityH";
        tStrings[recALat]    = "RxAccelerationLat";
        tStrings[recALon]    = "RxAccelerationLon";
        tStrings[recAH]      = "RxAccelerationH";
        tStrings[recJ2kX]    = "RxJ2kPositionX";
        tStrings[recJ2kY]    = "RxJ2kPositionY";
        tStrings[recJ2kZ]    = "RxJ2kPositionZ";
        tStrings[recJ2kVX]   = "RxJ2kVelocityX";
        tStrings[recJ2kVY]   = "RxJ2kVelocityY";
        tStrings[recJ2kVZ]   = "RxJ2kVelocityZ";
        tStrings[recJ2kAX]   = "RxJ2kAccelerationX";
        tStrings[recJ2kAY]   = "RxJ2kAccelerationY";
        tStrings[recJ2kAZ]   = "RxJ2kAccelerationZ";
        tStrings[sigma]      = "sigma";
        tStrings[iura]       = "iura";
        tStrings[Action]     = "Action";
        tStrings[corrRho]     = "correctedRange";
        tStrings[dummy1]     = "dummy1";
        tStrings[dummy2]     = "dummy2";
        tStrings[dummy3]     = "dummy3";
        tStrings[dummy4]     = "dummy4";
        tStrings[dummy5]     = "dummy5";
        tStrings[dummy6]     = "dummy6";
        tStrings[dummy7]     = "dummy7";
        tStrings[dummy8]     = "dummy8";
        tStrings[dummy9]     = "dummy9";
        tStrings[Last]       = "Last";
        tStrings[Placeholder]= "Placeholder";
}


// Assignment operator
TypeID TypeID::operator=(const TypeID& right)
{
        if ( this == &right ) return (*this);
        (*this).type = right.type;
        return *this;
}


// Convenience output method
std::ostream& TypeID::dump(std::ostream& s) const
{
        s << TypeID::tStrings[type];

        return s;
}    // TypeID::dump()


// Returns true if this is a valid TypeID. Basically just
// checks that the enum is defined
bool TypeID::isValid() const
{
        return !(type==Unknown);
}


/* Static method to add new TypeID's
 * @param string      Identifying string for the new TypeID
 */
TypeID::ValueType TypeID::newValueType(const std::string& s)
{
        ValueType newId =
                static_cast<ValueType>(TypeID::tStrings.rbegin()->first + 1);

        TypeID::tStrings[newId] = s;

        return newId;
}


namespace StringUtils
{

// convert this object to a string representation
std::string asString(const TypeID& p)
{
        std::ostringstream oss;
        p.dump(oss);
        return oss.str();
}

}     // End of namespace StringUtils


// stream output for TypeID
std::ostream& operator<<(std::ostream& s, const TypeID& p)
{
        p.dump(s);
        return s;
}


bool IsCarrierPhase(const RinexObsType& rot)
{
        return (rot.type[0]=='L') ? true : false;
}


int GetCarrierBand(const RinexObsType& rot)
{
        // 1 2 5 6 7 8
        try
        {
                return StringUtils::asInt( rot.type.substr(1,1) );
        }
        catch(...)
        {
                return -1;
        }
}

int GetCarrierBand(const RinexObsID& roi)
{
        // 1 2 5 6 7 8
        if(roi.band == ObsID::cbL1) return 1;
        if(roi.band == ObsID::cbG1) return 1;
        if(roi.band == ObsID::cbB1) return 1;

        if(roi.band == ObsID::cbL2) return 2;
        if(roi.band == ObsID::cbG2) return 2;
        if(roi.band == ObsID::cbB1) return 2;   // TD this is not correct

        if(roi.band == ObsID::cbL5) return 5;

        if(roi.band == ObsID::cbE6) return 6;
        if(roi.band == ObsID::cbB3) return 6;

        if(roi.band == ObsID::cbE5b) return 7;

        if(roi.band == ObsID::cbE5ab) return 8;

        return -1;
}

TypeID::ValueType ConvertToTypeID(const RinexObsType& rot,
                                  const RinexSatID& sat)
{
        if(sat.system==SatID::systemGPS)
        {
                //GPS     L1         1575.42     C1,P1       L1         D1         S1
                //        L2         1227.60     C2,P2       L2         D2         S2
                //        L5         1176.45      C5         L5         D5         S5

                // For L1: C1 P1 L1 D1 S1
                if(rot == RinexObsHeader::C1) return TypeID::C1;
                if(rot == RinexObsHeader::P1) return TypeID::P1;
                if(rot == RinexObsHeader::L1) return TypeID::L1;
                if(rot == RinexObsHeader::D1) return TypeID::D1;
                if(rot == RinexObsHeader::S1) return TypeID::S1;
                // For L2: C2 P2 L2 D2 S2
                if(rot == RinexObsHeader::C2) return TypeID::C2;
                if(rot == RinexObsHeader::P2) return TypeID::P2;
                if(rot == RinexObsHeader::L2) return TypeID::L2;
                if(rot == RinexObsHeader::D2) return TypeID::D2;
                if(rot == RinexObsHeader::S2) return TypeID::S2;
                // For L5: C5 L5 D5 S5
                if(rot == RinexObsHeader::C5) return TypeID::C5;
                if(rot == RinexObsHeader::L5) return TypeID::L5;
                if(rot == RinexObsHeader::D5) return TypeID::D5;
                if(rot == RinexObsHeader::S5) return TypeID::S5;
        }
        else if(sat.system==SatID::systemGlonass)
        {
                // Glonass G1         1602+k*9/16 C1,P1       L1         D1         S1
                //         G2         1246+k*7/16 C2,P2       L2         D2         S2

                // For L1: C1 P1 L1 D1 S1
                if(rot == RinexObsHeader::C1) return TypeID::C1;
                if(rot == RinexObsHeader::P1) return TypeID::P1;
                if(rot == RinexObsHeader::L1) return TypeID::L1;
                if(rot == RinexObsHeader::D1) return TypeID::D1;
                if(rot == RinexObsHeader::S1) return TypeID::S1;
                // For L2: C2 P2 L2 D2 S2
                if(rot == RinexObsHeader::C2) return TypeID::C2;
                if(rot == RinexObsHeader::P2) return TypeID::P2;
                if(rot == RinexObsHeader::L2) return TypeID::L2;
                if(rot == RinexObsHeader::D2) return TypeID::D2;
                if(rot == RinexObsHeader::S2) return TypeID::S2;
        }
        else if(sat.system==SatID::systemGalileo)
        {
                // Galileo E2-L1-E1   1575.42      C1         L1         D1         S1
                //         E5a        1176.45      C5         L5         D5         S5
                //         E5b        1207.140     C7         L7         D7         S7
                //         E5a+b      1191.795     C8         L8         D8         S8
                //         E6         1278.75      C6         L6         D6         S6
                // E2-L1-E1
                if(rot == RinexObsHeader::C1) return TypeID::C1;
                if(rot == RinexObsHeader::L1) return TypeID::L1;
                if(rot == RinexObsHeader::D1) return TypeID::D1;
                if(rot == RinexObsHeader::S1) return TypeID::S1;
                // E5a
                if(rot == RinexObsHeader::C5) return TypeID::C5;
                if(rot == RinexObsHeader::L5) return TypeID::L5;
                if(rot == RinexObsHeader::D5) return TypeID::D5;
                if(rot == RinexObsHeader::S5) return TypeID::S5;
                // E5b
                if(rot == RinexObsHeader::C7) return TypeID::C7;
                if(rot == RinexObsHeader::L7) return TypeID::L7;
                if(rot == RinexObsHeader::D7) return TypeID::D7;
                if(rot == RinexObsHeader::S7) return TypeID::S7;
                // E5a+b
                if(rot == RinexObsHeader::C8) return TypeID::C8;
                if(rot == RinexObsHeader::L8) return TypeID::L8;
                if(rot == RinexObsHeader::D8) return TypeID::D8;
                if(rot == RinexObsHeader::S8) return TypeID::S8;
                // E6
                if(rot == RinexObsHeader::C6) return TypeID::C6;
                if(rot == RinexObsHeader::L6) return TypeID::L6;
                if(rot == RinexObsHeader::D6) return TypeID::D6;
                if(rot == RinexObsHeader::S6) return TypeID::S6;
        }
        else if(sat.system==SatID::systemBeiDou)
        {
                // Compass E2   I/Q                 C2         L2         D2         S2
                //         E5b  I/Q                 C7         L7         D7         S7
                //         E6   I/Q                 C6         L6         D6         S6

                // For E2-B1
                if(rot == RinexObsHeader::C2) return TypeID::C2;
                if(rot == RinexObsHeader::L2) return TypeID::L2;
                if(rot == RinexObsHeader::D2) return TypeID::D2;
                if(rot == RinexObsHeader::S2) return TypeID::S2;
                // For E5b-B2
                if(rot == RinexObsHeader::C7) return TypeID::C7;
                if(rot == RinexObsHeader::L7) return TypeID::L7;
                if(rot == RinexObsHeader::D7) return TypeID::D7;
                if(rot == RinexObsHeader::S7) return TypeID::S7;
                // For E6-B3
                if(rot == RinexObsHeader::C6) return TypeID::C6;
                if(rot == RinexObsHeader::L6) return TypeID::L6;
                if(rot == RinexObsHeader::D6) return TypeID::D6;
                if(rot == RinexObsHeader::S6) return TypeID::S6;
        }
        else if(sat.system==SatID::systemGeosync)
        {
                // SBAS    L1         1575.42      C1         L1         D1         S1
                //         L5         1176.45      C5         L5         D5         S5

                // L1
                if(rot == RinexObsHeader::C1) return TypeID::C1;
                if(rot == RinexObsHeader::L1) return TypeID::L1;
                if(rot == RinexObsHeader::D1) return TypeID::D1;
                if(rot == RinexObsHeader::S1) return TypeID::S1;
                // L5
                if(rot == RinexObsHeader::C5) return TypeID::C5;
                if(rot == RinexObsHeader::L5) return TypeID::L5;
                if(rot == RinexObsHeader::D5) return TypeID::D5;
                if(rot == RinexObsHeader::S5) return TypeID::S5;
        }

        return TypeID::Unknown;
}


TypeID::ValueType ConvertToTypeID(const RinexObsID& roi,
                                  const RinexSatID& sat)
{
        if(sat.system==SatID::systemGPS)
        {
                //GPS     L1         1575.42     C1,P1       L1         D1         S1
                //        L2         1227.60     C2,P2       L2         D2         S2
                //        L5         1176.45      C5         L5         D5         S5

                // For L1: C1 P1 L1 D1 S1
                if(roi.band==ObsID::cbL1)
                {
                        if(roi.type == ObsID::otRange)
                                return (roi.code == ObsID::tcCA) ? TypeID::C1 : TypeID::P1;

                        if(roi.type == ObsID::otPhase) return TypeID::L1;
                        if(roi.type == ObsID::otDoppler) return TypeID::D1;
                        if(roi.type == ObsID::otSNR) return TypeID::S1;
                }
                // For L2: C2 P2 L2 D2 S2
                else if(roi.band==ObsID::cbL2)
                {
                        if(roi.type == ObsID::otRange)
                                return (roi.code == ObsID::tcCA) ? TypeID::C2 : TypeID::P2;

                        if(roi.type == ObsID::otPhase) return TypeID::L2;
                        if(roi.type == ObsID::otDoppler) return TypeID::D2;
                        if(roi.type == ObsID::otSNR) return TypeID::S2;
                }
                // For L5: C5 L5 D5 S5
                else if(roi.band==ObsID::cbL5)
                {
                        if(roi.type == ObsID::otRange) return TypeID::C5;
                        if(roi.type == ObsID::otPhase) return TypeID::L5;
                        if(roi.type == ObsID::otDoppler) return TypeID::D5;
                        if(roi.type == ObsID::otSNR) return TypeID::S5;
                }
        }
        else if(sat.system==SatID::systemGlonass)
        {
                // Glonass G1         1602+k*9/16 C1,P1       L1         D1         S1
                //         G2         1246+k*7/16 C2,P2       L2         D2         S2

                // For L1: C1 P1 L1 D1 S1
                if(roi.band == ObsID::cbG1)
                {
                        if(roi.type == ObsID::otRange) // tcGCA or tcGP
                                return (roi.code == ObsID::tcGCA) ? TypeID::C1 : TypeID::P1;

                        if(roi.type == ObsID::otPhase) return TypeID::L1;
                        if(roi.type == ObsID::otDoppler) return TypeID::D1;
                        if(roi.type == ObsID::otSNR) return TypeID::S1;
                }
                // For L2: C2 P2 L2 D2 S2
                else if(roi.band == ObsID::cbG1)
                {
                        if(roi.type == ObsID::otRange) // tcGCA or tcGP
                                return (roi.code == ObsID::tcGCA) ? TypeID::C2 : TypeID::P2;

                        if(roi.type == ObsID::otPhase) return TypeID::L2;
                        if(roi.type == ObsID::otDoppler) return TypeID::D2;
                        if(roi.type == ObsID::otSNR) return TypeID::S2;
                }
        }
        else if(sat.system==SatID::systemGalileo)
        {
                // Galileo E2-L1-E1   1575.42      C1         L1         D1         S1
                //         E5a        1176.45      C5         L5         D5         S5
                //         E5b        1207.140     C7         L7         D7         S7
                //         E5a+b      1191.795     C8         L8         D8         S8
                //         E6         1278.75      C6         L6         D6         S6
                // E2-L1-E1
                if(roi.band == ObsID::cbL1)  // E1
                {
                        if(roi.type == ObsID::otRange) return TypeID::C1;
                        if(roi.type == ObsID::otPhase) return TypeID::L1;
                        if(roi.type == ObsID::otDoppler) return TypeID::D1;
                        if(roi.type == ObsID::otSNR) return TypeID::S1;
                }
                else if(roi.band == ObsID::cbL5) // E5a
                {
                        if(roi.type == ObsID::otRange) return TypeID::C5;
                        if(roi.type == ObsID::otPhase) return TypeID::L5;
                        if(roi.type == ObsID::otDoppler) return TypeID::D5;
                        if(roi.type == ObsID::otSNR) return TypeID::S5;
                }
                else if(roi.band == ObsID::cbE5b) // E5b
                {
                        if(roi.type == ObsID::otRange) return TypeID::C7;
                        if(roi.type == ObsID::otPhase) return TypeID::L7;
                        if(roi.type == ObsID::otDoppler) return TypeID::D7;
                        if(roi.type == ObsID::otSNR) return TypeID::S7;
                }
                else if(roi.band == ObsID::cbE5ab) // E5a+b
                {
                        if(roi.type == ObsID::otRange) return TypeID::C8;
                        if(roi.type == ObsID::otPhase) return TypeID::L8;
                        if(roi.type == ObsID::otDoppler) return TypeID::D8;
                        if(roi.type == ObsID::otSNR) return TypeID::S8;
                }
                else if(roi.band == ObsID::cbE6) // E6
                {
                        if(roi.type == ObsID::otRange) return TypeID::C6;
                        if(roi.type == ObsID::otPhase) return TypeID::L6;
                        if(roi.type == ObsID::otDoppler) return TypeID::D6;
                        if(roi.type == ObsID::otSNR) return TypeID::S6;
                }
        }
        else if(sat.system==SatID::systemBeiDou)
        {
                // Compass E2   I/Q                 C2         L2         D2         S2
                //         E5b  I/Q                 C7         L7         D7         S7
                //         E6   I/Q                 C6         L6         D6         S6

                // For E2-B1
                //if(roi.band == ObsID::cbE1) return TypeID::Unknown;
                if(roi.band == ObsID::cbB3)  // TD is cbB3 correct?
                {
                        if(roi.type == ObsID::otRange) return TypeID::C2;
                        if(roi.type == ObsID::otPhase) return TypeID::L2;
                        if(roi.type == ObsID::otDoppler) return TypeID::D2;
                        if(roi.type == ObsID::otSNR) return TypeID::S2;
                }
                else if(roi.band == ObsID::cbE5b)
                {
                        if(roi.type == ObsID::otRange) return TypeID::C7;
                        if(roi.type == ObsID::otPhase) return TypeID::L7;
                        if(roi.type == ObsID::otDoppler) return TypeID::D7;
                        if(roi.type == ObsID::otSNR) return TypeID::S7;
                }
                else if(roi.band == ObsID::cbE6)
                {
                        if(roi.type == ObsID::otRange) return TypeID::C6;
                        if(roi.type == ObsID::otPhase) return TypeID::L6;
                        if(roi.type == ObsID::otDoppler) return TypeID::D6;
                        if(roi.type == ObsID::otSNR) return TypeID::S6;
                }
        }
        else if(sat.system==SatID::systemGeosync)
        {
                // SBAS    L1         1575.42      C1         L1         D1         S1
                //         L5         1176.45      C5         L5         D5         S5

                // L1
                if(roi.band == ObsID::cbL1)
                {
                        if(roi.type == ObsID::otRange) return TypeID::C1;
                        if(roi.type == ObsID::otPhase) return TypeID::L1;
                        if(roi.type == ObsID::otDoppler) return TypeID::D1;
                        if(roi.type == ObsID::otSNR) return TypeID::S1;
                }
                else if(roi.band == ObsID::cbL5)
                {
                        if(roi.type == ObsID::otRange) return TypeID::C5;
                        if(roi.type == ObsID::otPhase) return TypeID::L5;
                        if(roi.type == ObsID::otDoppler) return TypeID::D5;
                        if(roi.type == ObsID::otSNR) return TypeID::S5;
                }
        }

        return TypeID::Unknown;
}


// Return the new TypeID
TypeID TypeID::regByName(std::string name,std::string desc)
{

        std::map<std::string,TypeID>::iterator it = mapUserTypeID.find(name);

        if(it != mapUserTypeID.end())
        {
                return it->second;
        }
        else
        {
                TypeID newID = TypeID::newValueType(desc);

                mapUserTypeID.insert(std::pair<std::string,TypeID>(name, newID));

                return newID;
        }

}     // End of 'TypeID::registerTypeID(std::string name,std::string desc)'



// unregister a TypeID by it's name string
void TypeID::unregByName(std::string name)
{
        std::map<std::string,TypeID>::iterator it = mapUserTypeID.find(name);

        if(it!=mapUserTypeID.end())
        {
                TypeID delID = it->second;

                std::map<TypeID::ValueType,std::string>::iterator it2 = TypeID::tStrings.find(delID.type);
                if(it2!=TypeID::tStrings.end())
                {
                        TypeID::tStrings.erase(it2);
                }

                mapUserTypeID.erase(it);
        }
        else
        {
                // the TypeID have not been registered
                // we do nothing
        }

}    // End of 'TypeID::unregisterTypeID(std::string name)'



// unregister all TypeIDs registered by name string
void TypeID::unregAll()
{
        std::map<std::string,TypeID>::iterator it = mapUserTypeID.begin();

        for(it=mapUserTypeID.begin(); it!=mapUserTypeID.end(); it++)
        {
                TypeID delID = it->second;

                std::map<TypeID::ValueType,std::string>::iterator it2 = TypeID::tStrings.find(delID.type);
                if(it2!=TypeID::tStrings.end())
                {
                        TypeID::tStrings.erase(it2);
                }
        }
        mapUserTypeID.clear();

        bUserTypeIDRegistered = false;

}     // End of 'TypeID::unregisterAll()'

// get the user registered TypeID by name string
TypeID TypeID::byName(std::string name)
throw(InvalidRequest)
{
        // registerMyTypeID();

        std::map<std::string,TypeID>::iterator it = mapUserTypeID.find(name);
        if(it != mapUserTypeID.end())
        {
                return it->second;
        }
        else
        {
                InvalidRequest e("There are no registered TypeID name as '"
                                 + name + "'.");
                GPSTK_THROW(e);
        }
}    // End of 'TypeID TypeID::byName(std::string name)'

} // End of namespace gpstk
