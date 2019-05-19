/**
 * \file NETGeographicLib/CircularEngine.cpp
 * \brief Implementation for NETGeographicLib::CircularEngine class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/CircularEngine.hpp"
#include "CircularEngine.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
CircularEngine::CircularEngine(const GeographicLib::CircularEngine& c)
{
    try
    {
        m_pCircularEngine = new GeographicLib::CircularEngine( c );
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::CircularEngine" );
    }
}

//*****************************************************************************
CircularEngine::!CircularEngine()
{
    if ( m_pCircularEngine != NULL )
    {
        delete m_pCircularEngine;
        m_pCircularEngine = NULL;
    }
}

//*****************************************************************************
double CircularEngine::LongitudeSum(double coslon, double sinlon)
{
    return m_pCircularEngine->operator()( coslon, sinlon );
}

//*****************************************************************************
double CircularEngine::LongitudeSum(double lon)
{
    return m_pCircularEngine->operator()( lon );
}

//*****************************************************************************
double CircularEngine::LongitudeSum(double coslon, double sinlon,
                        [System::Runtime::InteropServices::Out] double% gradx,
                        [System::Runtime::InteropServices::Out] double% grady,
                        [System::Runtime::InteropServices::Out] double% gradz)
{
    double lgradx, lgrady, lgradz;
    double output = m_pCircularEngine->operator()( coslon, sinlon, lgradx, lgrady, lgradz );
    gradx = lgradx;
    grady = lgrady;
    gradz = lgradz;
    return output;
}

//*****************************************************************************
double CircularEngine::LongitudeSum(double lon,
                        [System::Runtime::InteropServices::Out] double% gradx,
                        [System::Runtime::InteropServices::Out] double% grady,
                        [System::Runtime::InteropServices::Out] double% gradz)
{
    double lgradx, lgrady, lgradz;
    double output = m_pCircularEngine->operator()( lon, lgradx, lgrady, lgradz );
    gradx = lgradx;
    grady = lgrady;
    gradz = lgradz;
    return output;
}
