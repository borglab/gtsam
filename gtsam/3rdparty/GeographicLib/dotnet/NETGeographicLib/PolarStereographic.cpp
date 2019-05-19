/**
 * \file NETGeographicLib/PolarStereographic.cpp
 * \brief Implementation for NETGeographicLib::PolarStereographic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/PolarStereographic.hpp"
#include "PolarStereographic.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::PolarStereographic";

//*****************************************************************************
PolarStereographic::!PolarStereographic(void)
{
    if ( m_pPolarStereographic != NULL )
    {
        delete m_pPolarStereographic;
        m_pPolarStereographic = NULL;
    }
}

//*****************************************************************************
PolarStereographic::PolarStereographic(double a, double f, double k0)
{
    try
    {
        m_pPolarStereographic = new GeographicLib::PolarStereographic( a, f, k0 );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
PolarStereographic::PolarStereographic()
{
    try
    {
        m_pPolarStereographic = new GeographicLib::PolarStereographic(
            GeographicLib::PolarStereographic::UPS() );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void PolarStereographic::SetScale(double lat, double k)
{
    try
    {
        m_pPolarStereographic->SetScale( lat, k );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void PolarStereographic::Forward(bool northp, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    m_pPolarStereographic->Forward( northp, lat, lon,
            lx, ly, lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void PolarStereographic::Reverse(bool northp, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    m_pPolarStereographic->Reverse( northp, x, y,
            llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void PolarStereographic::Forward(bool northp, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    m_pPolarStereographic->Forward( northp, lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void PolarStereographic::Reverse(bool northp, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pPolarStereographic->Reverse( northp, x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double PolarStereographic::MajorRadius::get()
{ return m_pPolarStereographic->MajorRadius(); }

//*****************************************************************************
double PolarStereographic::Flattening::get()
{ return m_pPolarStereographic->Flattening(); }

//*****************************************************************************
double PolarStereographic::CentralScale::get()
{ return m_pPolarStereographic->CentralScale(); }
