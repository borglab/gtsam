/**
 * \file NETGeographicLib/PolygonArea.cpp
 * \brief Implementation for NETGeographicLib::PolygonArea class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/PolygonArea.hpp"
#include "PolygonArea.h"
#include "Geodesic.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::PolygonArea";

//*****************************************************************************
PolygonArea::!PolygonArea(void)
{
    if ( m_pPolygonArea != NULL )
    {
        delete m_pPolygonArea;
        m_pPolygonArea = NULL;
    }
}

//*****************************************************************************
PolygonArea::PolygonArea(Geodesic^ earth, bool polyline )
{
    try
    {
        const GeographicLib::Geodesic* pGeodesic =
            reinterpret_cast<const GeographicLib::Geodesic*>(
                earth->GetUnmanaged()->ToPointer() );
        m_pPolygonArea = new GeographicLib::PolygonArea( *pGeodesic, polyline );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
PolygonArea::PolygonArea(const bool polyline )
{
    try
    {
        m_pPolygonArea = new GeographicLib::PolygonArea(
            GeographicLib::Geodesic::WGS84, polyline );
    }
    catch (std::bad_alloc)
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
void PolygonArea::Clear() { m_pPolygonArea->Clear(); }

//*****************************************************************************
void PolygonArea::AddPoint(double lat, double lon)
{
    m_pPolygonArea->AddPoint( lat, lon );
}

//*****************************************************************************
void PolygonArea::AddEdge(double azi, double s)
{
    m_pPolygonArea->AddEdge( azi, s );
}

//*****************************************************************************
unsigned PolygonArea::Compute(bool reverse, bool sign,
                    [System::Runtime::InteropServices::Out] double% perimeter,
                    [System::Runtime::InteropServices::Out] double% area)
{
    double lperimeter, larea;
    unsigned out = m_pPolygonArea->Compute( reverse, sign, lperimeter, larea );
    perimeter = lperimeter;
    area = larea;
    return out;
}

//*****************************************************************************
unsigned PolygonArea::TestPoint(double lat, double lon, bool reverse, bool sign,
                    [System::Runtime::InteropServices::Out] double% perimeter,
                    [System::Runtime::InteropServices::Out] double% area)
{
    double lperimeter, larea;
    unsigned out = m_pPolygonArea->TestPoint( lat, lon, reverse, sign, lperimeter, larea );
    perimeter = lperimeter;
    area = larea;
    return out;
}

//*****************************************************************************
unsigned PolygonArea::TestEdge(double azi, double s, bool reverse, bool sign,
                    [System::Runtime::InteropServices::Out] double% perimeter,
                    [System::Runtime::InteropServices::Out] double% area)
{
    double lperimeter, larea;
    unsigned out = m_pPolygonArea->TestEdge( azi, s, reverse, sign, lperimeter, larea );
    perimeter = lperimeter;
    area = larea;
    return out;
}

//*****************************************************************************
void PolygonArea::CurrentPoint(
    [System::Runtime::InteropServices::Out] double% lat,
    [System::Runtime::InteropServices::Out] double% lon)
{
    double llat, llon;
    m_pPolygonArea->CurrentPoint( llat, llon );
    lat = llat;
    lon = llon;
}

//*****************************************************************************
double PolygonArea::MajorRadius::get()
{ return m_pPolygonArea->MajorRadius(); }

//*****************************************************************************
double PolygonArea::Flattening::get() { return m_pPolygonArea->Flattening(); }
