/**
 * \file NETGeographicLib/TransverseMercator.cpp
 * \brief Implementation for NETGeographicLib::TransverseMercator class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/TransverseMercator.hpp"
#include "TransverseMercator.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::TransverseMercator";

//*****************************************************************************
TransverseMercator::!TransverseMercator(void)
{
    if ( m_pTransverseMercator != NULL )
    {
        delete m_pTransverseMercator;
        m_pTransverseMercator = NULL;
    }
}

//*****************************************************************************
TransverseMercator::TransverseMercator(double a, double f, double k0)
{
    try
    {
        m_pTransverseMercator = new GeographicLib::TransverseMercator( a, f, k0 );
    }
    catch (std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch (std::exception err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
TransverseMercator::TransverseMercator()
{
    try
    {
        m_pTransverseMercator = new GeographicLib::TransverseMercator(
            GeographicLib::TransverseMercator::UTM() );
    }
    catch (std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void TransverseMercator::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double lx, ly, lgamma, lk;
    m_pTransverseMercator->Forward( lon0, lat, lon, lx, ly, lgamma, lk );
    x = lx;
    y = ly;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void TransverseMercator::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon,
                [System::Runtime::InteropServices::Out] double% gamma,
                [System::Runtime::InteropServices::Out] double% k)
{
    double llat, llon, lgamma, lk;
    m_pTransverseMercator->Reverse( lon0, x, y, llat, llon, lgamma, lk );
    lat = llat;
    lon = llon;
    gamma = lgamma;
    k = lk;
}

//*****************************************************************************
void TransverseMercator::Forward(double lon0, double lat, double lon,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y)
{
    double lx, ly;
    m_pTransverseMercator->Forward( lon0, lat, lon, lx, ly );
    x = lx;
    y = ly;
}

//*****************************************************************************
void TransverseMercator::Reverse(double lon0, double x, double y,
                [System::Runtime::InteropServices::Out] double% lat,
                [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pTransverseMercator->Reverse( lon0, x, y, llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double TransverseMercator::MajorRadius::get()
{ return m_pTransverseMercator->MajorRadius(); }

//*****************************************************************************
double TransverseMercator::Flattening::get()
{ return m_pTransverseMercator->Flattening(); }

//*****************************************************************************
double TransverseMercator::CentralScale::get()
{ return m_pTransverseMercator->CentralScale(); }
