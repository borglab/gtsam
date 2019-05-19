/**
 * \file NETGeographicLib/Accumulator.cpp
 * \brief Implementation for NETGeographicLib::Accumulator class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Accumulator.hpp"
#include "Accumulator.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
Accumulator::!Accumulator(void)
{
    if ( m_pAccumulator != NULL )
    {
        delete m_pAccumulator;
        m_pAccumulator = NULL;
    }
}

//*****************************************************************************
Accumulator::Accumulator(void)
{
    try
    {
        m_pAccumulator = new GeographicLib::Accumulator<double>();
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr("Failed to allocate memory for a GeogrpicLib::Accumulator");
    }
}

//*****************************************************************************
void Accumulator::Assign( double a )
{
    m_pAccumulator->operator=( a);
}

//*****************************************************************************
double Accumulator::Result()
{
    return m_pAccumulator->operator()();
}

//*****************************************************************************
void Accumulator::Sum( double a )
{
    m_pAccumulator->operator+=( a );
}

//*****************************************************************************
void Accumulator::Multiply( int i )
{
    m_pAccumulator->operator*=( i );
}

//*****************************************************************************
bool Accumulator::operator == ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator==( a );
}

//*****************************************************************************
bool Accumulator::operator != ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator!=( a );
}

//*****************************************************************************
bool Accumulator::operator < ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator<( a );
}

//*****************************************************************************
bool Accumulator::operator <= ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator<=( a );
}

//*****************************************************************************
bool Accumulator::operator > ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator>( a );
}

//*****************************************************************************
bool Accumulator::operator >= ( Accumulator^ lhs, double a )
{
    return lhs->m_pAccumulator->operator>=( a );
}
