/**
 * \file NETGeographicLib/Rhumb.cpp
 * \brief Implementation for NETGeographicLib::Rhumb and NETGeographicLib::RhumbLine class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Rhumb.hpp"
#include "Rhumb.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
Rhumb::!Rhumb(void)
{
    if ( m_pRhumb != NULL )
    {
        delete m_pRhumb;
        m_pRhumb = NULL;
    }
}

//*****************************************************************************
Rhumb::Rhumb(double a, double f, bool exact)
{
    try
    {
        m_pRhumb = new GeographicLib::Rhumb( a, f, exact );
    }
    catch ( GeographicLib::GeographicErr& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew System::Exception("Failed to allocate memory for a Rhumb.");
    }
}

//*****************************************************************************
void Rhumb::Direct(double lat1, double lon1, double azi12, double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% S12)
{
    double ilat2, ilon2, iS12;
    m_pRhumb->Direct( lat1, lon1, azi12, s12, ilat2, ilon2, iS12 );
    lat2 = ilat2;
    lon2 = ilon2;
    S12 = iS12;
}

//*****************************************************************************
void Rhumb::Direct(double lat1, double lon1, double azi12, double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2)
{
    double ilat2, ilon2;
    m_pRhumb->Direct( lat1, lon1, azi12, s12, ilat2, ilon2 );
    lat2 = ilat2;
    lon2 = ilon2;
}

//*****************************************************************************
void Rhumb::GenDirect(double lat1, double lon1, double azi12, double s12,
                Rhumb::mask outmask,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% S12)
{
    double ilat2, ilon2, iS12;
    unsigned int iMask = (unsigned int)outmask;
    m_pRhumb->GenDirect( lat1, lon1, azi12, s12, iMask, ilat2, ilon2, iS12 );
    lat2 = ilat2;
    lon2 = ilon2;
    S12 = iS12;
}

//*****************************************************************************
void Rhumb::Inverse(double lat1, double lon1, double lat2, double lon2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% azi12,
            [System::Runtime::InteropServices::Out] double% S12)
{
    double is12, iazi12, iS12;
    m_pRhumb->Inverse( lat1, lon1, lat2, lon2, is12, iazi12, iS12 );
    s12 = is12;
    azi12 = iazi12;
    S12 = iS12;
}

//*****************************************************************************
void Rhumb::Inverse(double lat1, double lon1, double lat2, double lon2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% azi12)
{
    double is12, iazi12;
    m_pRhumb->Inverse( lat1, lon1, lat2, lon2, is12, iazi12 );
    s12 = is12;
    azi12 = iazi12;
}

//*****************************************************************************
void Rhumb::GenInverse(double lat1, double lon1, double lat2, double lon2,
                Rhumb::mask outmask,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% azi12,
                [System::Runtime::InteropServices::Out] double% S12)
{
    double is12, iazi12, iS12;
    unsigned int iMask = (unsigned int)outmask;
    m_pRhumb->GenInverse( lat1, lon1, lat2, lon2, iMask, is12, iazi12, iS12 );
    s12 = is12;
    azi12 = iazi12;
    S12 = iS12;
}

//*****************************************************************************
RhumbLine^ Rhumb::Line(double lat1, double lon1, double azi12)
{
    return gcnew RhumbLine( new GeographicLib::RhumbLine(m_pRhumb->Line( lat1, lon1, azi12 )) );
}

//*****************************************************************************
double Rhumb::MajorRadius::get() { return m_pRhumb->MajorRadius(); }

//*****************************************************************************
double Rhumb::Flattening::get() { return m_pRhumb->Flattening(); }

//*****************************************************************************
double Rhumb::EllipsoidArea::get() { return m_pRhumb->EllipsoidArea(); }

//*****************************************************************************
Rhumb^ Rhumb::WGS84()
{
    return gcnew Rhumb( GeographicLib::Constants::WGS84_a(),
                        GeographicLib::Constants::WGS84_f(), false );
}

//*****************************************************************************
System::IntPtr^ Rhumb::GetUnmanaged()
{
    return gcnew System::IntPtr( const_cast<void*>(reinterpret_cast<const void*>(m_pRhumb)) );
}

//*****************************************************************************
// RhumbLine functions
//*****************************************************************************
RhumbLine::!RhumbLine(void)
{
    if ( m_pRhumbLine != NULL )
    {
        delete m_pRhumbLine;
        m_pRhumbLine = NULL;
    }
}

//*****************************************************************************
RhumbLine::RhumbLine( GeographicLib::RhumbLine* pRhumbLine )
{
    if ( pRhumbLine == NULL )
        throw gcnew System::Exception("Invalid pointer in RhumbLine constructor.");
    m_pRhumbLine = pRhumbLine;
}

//*****************************************************************************
void RhumbLine::Position(double s12,
              [System::Runtime::InteropServices::Out] double% lat2,
              [System::Runtime::InteropServices::Out] double% lon2,
              [System::Runtime::InteropServices::Out] double% S12)
{
    double ilat2, ilon2, iS12;
    m_pRhumbLine->Position( s12, ilat2, ilon2, iS12);
    lat2 = ilat2;
    lon2 = ilon2;
    S12 = iS12;
}

//*****************************************************************************
void RhumbLine::Position(double s12,
        [System::Runtime::InteropServices::Out] double% lat2,
        [System::Runtime::InteropServices::Out] double% lon2)
{
    double ilat2, ilon2;
    m_pRhumbLine->Position( s12, ilat2, ilon2 );
    lat2 = ilat2;
    lon2 = ilon2;
}

//*****************************************************************************
void RhumbLine::GenPosition(double s12, RhumbLine::mask outmask,
                 [System::Runtime::InteropServices::Out] double% lat2,
                 [System::Runtime::InteropServices::Out] double% lon2,
                 [System::Runtime::InteropServices::Out] double% S12)
{
    double ilat2, ilon2, iS12;
    unsigned int iMask = (unsigned int)outmask;
    m_pRhumbLine->GenPosition( s12, iMask, ilat2, ilon2, iS12);
    lat2 = ilat2;
    lon2 = ilon2;
    S12 = iS12;
}

//*****************************************************************************
double RhumbLine::Latitude::get()
{
    return m_pRhumbLine->Latitude();
}

//*****************************************************************************
double RhumbLine::Longitude::get()
{
    return m_pRhumbLine->Longitude();
}

//*****************************************************************************
double RhumbLine::Azimuth::get()
{
    return m_pRhumbLine->Azimuth();
}

//*****************************************************************************
double RhumbLine::MajorRadius::get()
{
    return m_pRhumbLine->MajorRadius();
}

//*****************************************************************************
double RhumbLine::Flattening::get()
{
    return m_pRhumbLine->Flattening();
}
