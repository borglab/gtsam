/**
 * \file NETGeographicLib/TransverseMercatorExact.cpp
 * \brief Implementation for NETGeographicLib::TransverseMercatorExact class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/TransverseMercatorExact.hpp"
#include "TransverseMercatorExact.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Unable to allocate memory for a GeographicLib::TransverseMercatorExact";

//*****************************************************************************
TransverseMercatorExact::!TransverseMercatorExact(void)
{
    if ( m_pTransverseMercatorExact != NULL )
    {
        delete m_pTransverseMercatorExact;
        m_pTransverseMercatorExact = NULL;
    }
}

//*****************************************************************************
TransverseMercatorExact::TransverseMercatorExact(double a, double f, double k0,
    bool extendp)
{
    try
    {
        m_pTransverseMercatorExact = new GeographicLib::TransverseMercatorExact(
            a, f, k0, extendp );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( std::exception err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
TransverseMercatorExact::TransverseMercatorExact()
{
    try
    {
        m_pTransverseMercatorExact =
            new GeographicLib::TransverseMercatorExact(
                GeographicLib::TransverseMercatorExact::UTM() );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void TransverseMercatorExact::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    m_pTransverseMercatorExact->Forward( lon0, lat, lon, lx, ly,
                                            lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void TransverseMercatorExact::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    m_pTransverseMercatorExact->Reverse( lon0, x, y, llat, llon,
                                            lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void TransverseMercatorExact::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    m_pTransverseMercatorExact->Forward( lon0, lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void TransverseMercatorExact::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pTransverseMercatorExact->Reverse( lon0, x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double TransverseMercatorExact::MajorRadius::get()
{ return m_pTransverseMercatorExact->MajorRadius(); }

//*****************************************************************************
double TransverseMercatorExact::Flattening::get()
{ return m_pTransverseMercatorExact->Flattening(); }

//*****************************************************************************
double TransverseMercatorExact::CentralScale::get()
{ return m_pTransverseMercatorExact->CentralScale(); }
