/**
 * \file NETGeographicLib/GeoCoords.cpp
 * \brief Implementation for NETGeographicLib::GeoCoords class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GeoCoords.hpp"
#include "GeoCoords.h"
#include "UTMUPS.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::GeoCoords";

//*****************************************************************************
GeoCoords::!GeoCoords(void)
{
    if ( m_pGeoCoords != NULL )
    {
        delete m_pGeoCoords;
        m_pGeoCoords = NULL;
    }
}

//*****************************************************************************
GeoCoords::GeoCoords()
{
    try
    {
        m_pGeoCoords = new GeographicLib::GeoCoords();
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
GeoCoords::GeoCoords(System::String^ s, bool centerp, bool longfirst )
{
    try
    {
        m_pGeoCoords = new GeographicLib::GeoCoords(StringConvert::ManagedToUnmanaged(s), centerp, longfirst);
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
GeoCoords::GeoCoords(double latitude, double longitude, int zone )
{
    try
    {
        m_pGeoCoords = new GeographicLib::GeoCoords(latitude, longitude, zone);
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
GeoCoords::GeoCoords(int zone, bool northp, double easting, double northing)
{
    try
    {
        m_pGeoCoords = new GeographicLib::GeoCoords(zone, northp, easting, northing);
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
void GeoCoords::Reset( System::String^ s, bool centerp, bool longfirst )
{
    try
    {
        m_pGeoCoords->Reset(StringConvert::ManagedToUnmanaged(s), centerp, longfirst);
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void GeoCoords::Reset(double latitude, double longitude, int zone)
{
    try
    {
        m_pGeoCoords->Reset(latitude, longitude, zone);
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void GeoCoords::Reset(int zone, bool northp, double easting, double northing)
{
    try
    {
        m_pGeoCoords->Reset(zone, northp, easting, northing);
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void GeoCoords::AltZone::set( int zone )
{
    try
    {
        m_pGeoCoords->SetAltZone(zone);
    }
    catch ( GeographicLib::GeographicErr err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
int GeoCoords::AltZone::get() { return m_pGeoCoords->AltZone(); }

//*****************************************************************************
double GeoCoords::Latitude::get() { return m_pGeoCoords->Latitude(); }

//*****************************************************************************
double GeoCoords::Longitude::get() { return m_pGeoCoords->Longitude(); }

//*****************************************************************************
double GeoCoords::Easting::get() { return m_pGeoCoords->Easting(); }

//*****************************************************************************
double GeoCoords::Northing::get() { return m_pGeoCoords->Northing(); }

//*****************************************************************************
double GeoCoords::Convergence::get() { return m_pGeoCoords->Convergence(); }

//*****************************************************************************
double GeoCoords::Scale::get() { return m_pGeoCoords->Scale(); }

//*****************************************************************************
bool GeoCoords::Northp::get() { return m_pGeoCoords->Northp(); }

//*****************************************************************************
char GeoCoords::Hemisphere::get() { return m_pGeoCoords->Hemisphere(); }

//*****************************************************************************
int GeoCoords::Zone::get() { return m_pGeoCoords->Zone(); }

//*****************************************************************************
double GeoCoords::AltEasting::get() { return m_pGeoCoords->AltEasting(); }

//*****************************************************************************
double GeoCoords::AltNorthing::get() { return m_pGeoCoords->AltNorthing(); }

//*****************************************************************************
double GeoCoords::AltConvergence::get()
{ return m_pGeoCoords->AltConvergence(); }

//*****************************************************************************
double GeoCoords::AltScale::get() { return m_pGeoCoords->AltScale(); }

//*****************************************************************************
double GeoCoords::MajorRadius::get() { return UTMUPS::MajorRadius(); }

//*****************************************************************************
double GeoCoords::Flattening::get() { return UTMUPS::Flattening(); }

//*****************************************************************************
System::String^ GeoCoords::GeoRepresentation(int prec, bool longfirst )
{
    return gcnew System::String( m_pGeoCoords->GeoRepresentation(prec, longfirst).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::DMSRepresentation(int prec, bool longfirst,
                                char dmssep )
{
    return gcnew System::String( m_pGeoCoords->DMSRepresentation(prec, longfirst, dmssep).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::MGRSRepresentation(int prec)
{
    return gcnew System::String( m_pGeoCoords->MGRSRepresentation(prec).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::UTMUPSRepresentation(int prec, bool abbrev)
{
    return gcnew System::String( m_pGeoCoords->UTMUPSRepresentation(prec, abbrev).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::UTMUPSRepresentation(bool northp, int prec, bool abbrev)
{
    return gcnew System::String( m_pGeoCoords->UTMUPSRepresentation(northp, prec, abbrev).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::AltMGRSRepresentation(int prec)
{
    return gcnew System::String( m_pGeoCoords->AltMGRSRepresentation(prec).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::AltUTMUPSRepresentation(int prec, bool abbrev)
{
    return gcnew System::String( m_pGeoCoords->AltUTMUPSRepresentation(prec, abbrev).c_str() );
}

//*****************************************************************************
System::String^ GeoCoords::AltUTMUPSRepresentation(bool northp, int prec, bool abbrev)
{
    return gcnew System::String( m_pGeoCoords->AltUTMUPSRepresentation(northp, prec, abbrev).c_str() );
}
