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

#ifndef GPSTK_CORRECT_CODE_BIASES_HPP
#define GPSTK_CORRECT_CODE_BIASES_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/DCBDataReader.hpp>
#include <string>

namespace gpstk
{

/// @ingroup DataStructures
//@{

/** This class corrects observables from differential code
 * biases.(P1-P2) or (P1-C1)
 *
 * This class is meant to be used with the GNSS data structures objects
 * found in "DataStructures" class.
 *
 * A typical way to use this class follows:
 *
 * @code
 *
 *
 *   gnssRinex gRin;
 *   CorrectCodeBiases corr;
 *   coor.setDCBFile("P1P21001_ALL.DCB", "P1C11001.DCB");
 *
 *   while(rin >> gRin)
 *   {
 *      gRin >> corr;
 *   }
 *
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
class CorrectCodeBiases : public ProcessingClass
{
public:

/// Default constructor
CorrectCodeBiases();

/// Default deconstructor
virtual ~CorrectCodeBiases();

/** Sets name of file containing DCBs data.
 * @param name      Name of the file containing DCB(P1-P2)
 * @param name      Name of the file containing DCB(P1-C1)
 */
virtual CorrectCodeBiases& setDCBFile( const std::string& fileP1P2,
                                       const std::string& fileP1C1);

/** Set if C1 has been used as P1 to calculate some combinations
 * @param useC1      If C1 has been used as P1, then set it to true
 */
virtual CorrectCodeBiases& setUsingC1(const bool& useC1)
{
        usingC1 = useC1; return (*this);
}


/** Set receiver name
 * @param receiver      Name of receiver with 4 char
 */
virtual CorrectCodeBiases& setReceiver(const std::string& receiver)
{
        receiverName = receiver; return (*this);
}


/** Returns a satTypeValueMap object, adding the new data generated
 *  when calling this object.
 *
 * @param time      Epoch corresponding to the data.
 * @param gData     Data object holding the data.
 */
virtual satTypeValueMap& Process( const CommonTime& time,
                                  satTypeValueMap& gData )
throw(ProcessingException);


/** Returns a gnnsSatTypeValue object, adding the new data
 *  generated when calling this object.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
throw(ProcessingException)
{
        Process(gData.header.epoch, gData.body); return gData;
};


/** Returns a gnnsRinex object, adding the new data generated
 *  when calling this object.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssRinex& Process(gnssRinex& gData)
throw(ProcessingException)
{
        Process(gData.header.epoch, gData.body); return gData;
};


/// Returns a string identifying this object.
virtual std::string getClassName() const;


protected:

/// get DCB(Differental Code Biases) corrections
virtual double getDCBCorrection(const std::string& receiver,
                                const SatID&  sat,
                                const TypeID& type,
                                const bool&   useC1 = false);

DCBDataReader dcbP1P2;
DCBDataReader dcbP1C1;

/// if C1 is used as P1 to calculate some combination
bool usingC1;

/// receiver name
std::string receiverName;

/// If it's a cross-correlation receiver, set it true
// it's false by default
bool crossCorrelationReceiver;


private:


/// Factors
const static double factoP1P2[6];
const static double factorP1C1[6];
const static double factorC1X2[6];        // for cross-correlation receiver

};    // End of class 'CorrectCodeBiases'

//@}

}  // End of namespace gpstk

#endif   // GPSTK_CORRECT_CODE_BIASES_HPP
