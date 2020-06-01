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
//  Wei Yan - Chinese Academy of Sciences . 2009, 2010, 2011
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
 * @file CorrectCodeBiases.hpp
 *
 */

#include <gtsam/gpstk/CorrectCodeBiases.hpp>
#include <gtsam/gpstk/GNSSconstants.hpp>

namespace gpstk
{

const double CorrectCodeBiases::factoP1P2[6] = {
        +L2_FREQ_GPS*L2_FREQ_GPS/(L1_FREQ_GPS*L1_FREQ_GPS - L2_FREQ_GPS*L2_FREQ_GPS), // L1
        +L1_FREQ_GPS*L1_FREQ_GPS/(L1_FREQ_GPS*L1_FREQ_GPS - L2_FREQ_GPS*L2_FREQ_GPS), // L2
        +0.0,                                                // L3
        -1.0,                                                // L4
        -L1_FREQ_GPS*L2_FREQ_GPS/(L1_FREQ_GPS*L1_FREQ_GPS - L2_FREQ_GPS*L2_FREQ_GPS), // L5
        0.0                                                 // L6
};

const double CorrectCodeBiases::factorP1C1[6]={
        +1.0,                                                 // L1
        +0.0,                                                 // L2
        +L1_FREQ_GPS*L1_FREQ_GPS/(L1_FREQ_GPS*L1_FREQ_GPS - L2_FREQ_GPS*L2_FREQ_GPS), // L3
        +1.0,                                                 // L4
        +L1_FREQ_GPS/(L1_FREQ_GPS-L2_FREQ_GPS),                           // L5
        -L1_FREQ_GPS/(L1_FREQ_GPS+L2_FREQ_GPS)                            // L6
};

const double CorrectCodeBiases::factorC1X2[6]={
        +1.0,
        +1.0,
        +1.0,
        +0.0,
        +1.0,
        -1.0
};

// Returns a string identifying this object.
std::string CorrectCodeBiases::getClassName() const
{
        return "CorrectCodeBiases";
}

// Default constructor
CorrectCodeBiases::CorrectCodeBiases()
        : usingC1(false),
        receiverName(""),
        crossCorrelationReceiver(false)
{
}

// Default deconstructor
CorrectCodeBiases::~CorrectCodeBiases()
{
        // make sure the files are closed
        dcbP1P2.close();
        dcbP1C1.close();
}

/* Sets name of file containing DCBs data.
 * @param name      Name of the file containing DCB(P1-P2)
 * @param name      Name of the file containing DCB(P1-C1)
 */
CorrectCodeBiases& CorrectCodeBiases::setDCBFile( const std::string& fileP1P2,
                                                  const std::string& fileP1C1)
{
        dcbP1P2.open(fileP1P2);
        dcbP1C1.open(fileP1C1);

        return (*this);
}

/* Returns a satTypeValueMap object, adding the new data generated
 *  when calling this object.
 *
 * @param time      Epoch corresponding to the data.
 * @param gData     Data object holding the data.
 */
satTypeValueMap& CorrectCodeBiases::Process( const CommonTime& time,
                                             satTypeValueMap& gData )
throw(ProcessingException)
{
        try
        {
                SatIDSet satRejectedSet;

                // Loop through all the satellites
                satTypeValueMap::iterator it;
                for (it = gData.begin(); it != gData.end(); ++it)
                {
                        SatID sat = it->first;
                        for(typeValueMap::iterator itt = it->second.begin();
                            itt != it->second.end();
                            ++itt)
                        {
                                TypeID type = itt->first;
                                //itt->second += getDCBCorrection(receiverName, sat, type, usingC1);

                                if( (type == TypeID::C1) || (type == TypeID::P1))
                                {
                                        gData[sat][TypeID::instC1] = getDCBCorrection(receiverName, sat, type, usingC1);
                                }
                                else if( (type == TypeID::P2) )
                                {
                                        gData[sat][TypeID::instC2] = getDCBCorrection(receiverName, sat, type, usingC1);
                                }

                        }

                } // End of 'for (it = gData.begin(); it != gData.end(); ++it)'

                // Remove satellites with missing data
                gData.removeSatID(satRejectedSet);

                return gData;

        }
        catch(Exception& u)
        {
                // Throw an exception if something unexpected happens
                ProcessingException e( getClassName() + ":"
                                       + u.what() );

                GPSTK_THROW(e);

        }


}     // End of method 'CorrectCodeBiases::Process()'

double CorrectCodeBiases::getDCBCorrection(const std::string& receiver,
                                           const SatID&  sat,
                                           const TypeID& type,
                                           const bool&   useC1)
{
        double satP1P2(0.0);
        double satP1C1(0.0);
        double receiverP1P2(0.0);

        try
        {
                satP1P2 = dcbP1P2.getDCB(sat);
                satP1C1 = dcbP1C1.getDCB(sat);
                receiverP1P2 = dcbP1P2.getDCB(receiver, SatID::systemGPS);
        }
        catch(...)
        {
                // exception
                satP1P2 = 0.0;
                satP1C1 = 0.0;
        }

        int ind = -1;

        if( (type == TypeID::C1) ||
            (type == TypeID::P1) ||
            (type == TypeID::GRAPHIC1) )
        {
                ind = 0;
        }
        else if( (type == TypeID::P2)       ||
                 (type == TypeID::GRAPHIC2) )
        {
                ind = 1;
        }
        else if( type == TypeID::PC )
        {
                ind = 2;
        }
        else if( type == TypeID::PI)
        {
                ind = 3;
        }
        else if( type == TypeID::MWubbena )
        {
                ind = 5;
        }
        else
        {
                ind = -1;

                return 0.0;
        }

        double dcb1(0.0), dcb2(0.0), dcb3(0.0);
        if(ind >= 0)
        {
                dcb1 = factoP1P2[ind];
                dcb2 = factorP1C1[ind];
                dcb3 = factorC1X2[ind];
        }

        if( !useC1 && (type==TypeID::P1))
        {
                dcb2 = 0.0;
        }

        if(crossCorrelationReceiver)
        {
                dcb2 = dcb3;
        }

        double dcb =  dcb1 * (satP1P2 + receiverP1P2) + dcb2 * satP1C1;
        double corr = -1.0 * dcb * (C_MPS * 1.0e-9);  // ns -> meter
        return corr;
}     // End of method 'CorrectCodeBiases::getDCBCorrection()'


}  // End of namespace gpstk
