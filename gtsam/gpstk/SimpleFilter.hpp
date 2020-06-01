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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2009, 2011
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
 * @file SimpleFilter.hpp
 * This class filters out satellites with observations grossly out of bounds.
 */

#ifndef GPSTK_SIMPLEFILTER_HPP
#define GPSTK_SIMPLEFILTER_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>


namespace gpstk
{

/** @addtogroup DataStructures */
//@{


/** This class filters out satellites with observations grossly out of
 *  bounds.
 *
 * This class is meant to be used with the GNSS data structures objects
 * found in "DataStructures" class.
 *
 * A typical way to use this class follows:
 *
 * @code
 *   RinexObsStream rin("ebre0300.02o");
 *
 *   gnssRinex gRin;
 *   SimpleFilter myFilter;
 *
 *   while(rin >> gRin)
 *   {
 *      gRin >> myFilter;
 *   }
 * @endcode
 *
 * The "SimpleFilter" object will visit every satellite in the GNSS data
 * structure that is "gRin" and will check that the given code
 * observations are within some (preassigned) boundaries.
 *
 * By default, the algorithm will check C1 observables, the minimum limit
 * is 15000000.0 meters and the maximum limit is 30000000.0 meters. You
 * may change all these settings with the appropriate set methods.
 *
 * Also, you may set more than one observable to be checked by passing a
 * "TypeIDSet" object to the appropriate constructors or methods. For
 * instance:
 *
 * @code
 *   TypeIDSet typeSet;
 *   typeSet.insert(TypeID::P1);
 *   typeSet.insert(TypeID::P2);
 *
 *   myFilter.setFilteredType(typeSet);
 * @endcode
 *
 * Be warned that if a given satellite does not have the observations
 * required, or if their are out of bounds, the full satellite record
 * will be summarily deleted from the data structure.
 *
 */
class SimpleFilter : public ProcessingClass
{
public:

/// Default constructor. By default, filter C1.
SimpleFilter() : minLimit(15000000.0), maxLimit(30000000.0)
{
        setFilteredType(TypeID::C1);
};


/** Explicit constructor
 *
 * @param type      TypeID to be filtered.
 * @param min       Minimum limit (in meters).
 * @param max       Maximum limit (in meters).
 */
SimpleFilter( const TypeID& type,
              const double& min,
              const double& max )
        : minLimit(min), maxLimit(max)
{
        setFilteredType(type);
};


/** Explicit constructor
 *
 * @param type      TypeID to be filtered.
 */
SimpleFilter(const TypeID& type)
        : minLimit(15000000.0), maxLimit(30000000.0)
{
        setFilteredType(type);
};


/** Explicit constructor
 *
 * @param typeSet   Set of TypeID's to be filtered.
 * @param min       Minimum limit (in meters).
 * @param max       Maximum limit (in meters).
 */
SimpleFilter( const TypeIDSet& typeSet,
              const double& min,
              const double& max )
        : filterTypeSet(typeSet), minLimit(min), maxLimit(max)
{
};


/** Explicit constructor
 *
 * @param typeSet   Set of TypeID's to be filtered.
 */
SimpleFilter(const TypeIDSet& typeSet)
        : filterTypeSet(typeSet), minLimit(15000000.0), maxLimit(30000000.0)
{
};


/** Returns a satTypeValueMap object, filtering the target
 *  observables.
 *
 * @param gData     Data object holding the data.
 */
virtual satTypeValueMap& Process(satTypeValueMap& gData)
throw(ProcessingException);


/** Method to set the minimum limit.
 * @param min       Minimum limit (in meters).
 */
virtual SimpleFilter& setMinLimit(const double& min)
{
        minLimit = min; return (*this);
};


/// Method to get the minimum limit.
virtual double getMinLimit() const
{
        return minLimit;
};


/** Method to set the maximum limit.
 * @param max       Maximum limit (in meters).
 */
virtual SimpleFilter& setMaxLimit(const double& max)
{
        maxLimit = max; return (*this);
};


/// Method to get the maximum limit.
virtual double getMaxLimit() const
{
        return maxLimit;
};


/** Method to add a TypeID to be filtered.
 * @param type      Extra TypeID to be filtered.
 */
virtual SimpleFilter& addFilteredType(const TypeID& type)
{
        filterTypeSet.insert(type); return (*this);
};


/** Method to set a TypeID to be filtered. This method will erase
 *  previous types.
 * @param type      TypeID to be filtered.
 */
virtual SimpleFilter& setFilteredType(const TypeID& type)
{
        filterTypeSet.clear(); filterTypeSet.insert(type); return (*this);
};


/** Method to set the TypeID's to be filtered. This method will erase
 *  previous types.
 * @param typeSet       Set of TypeID's to be filtered.
 */
virtual SimpleFilter& setFilteredType(const TypeIDSet& typeSet)
{
        filterTypeSet.clear(); filterTypeSet = typeSet; return (*this);
};


/// Method to get the set of TypeID's to be filtered.
virtual TypeIDSet getFilteredType() const
{
        return filterTypeSet;
};


/** Returns a gnnsSatTypeValue object, filtering the target
 *  observables.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
throw(ProcessingException)
{
        Process(gData.body); return gData;
};



/** Returns a gnnsRinex object, filtering the target observables.
 *
 * @param gData    Data object holding the data.
 */
virtual gnssRinex& Process(gnssRinex& gData)
throw(ProcessingException)
{
        Process(gData.body); return gData;
};


/// Returns a string identifying this object.
virtual std::string getClassName(void) const;


/// Destructor
virtual ~SimpleFilter() {
};


protected:


/** Checks that the value is within the given limits.
 * @param value     The value to be test
 *
 * @return
 *  True if check was OK.
 */
virtual bool checkValue(const double& value) const
{
        return ( (value>=minLimit) && (value<=maxLimit) );
};


/// Set of types to be filtered
TypeIDSet filterTypeSet;

/// Minimum value allowed for input data (in meters).
double minLimit;

/// Maximum value allowed for input data (in meters).
double maxLimit;


};    // End of class 'SimpleFilter'

//@}

}  // End of namespace gpstk

#endif   // GPSTK_SIMPLEFILTER_HPP
