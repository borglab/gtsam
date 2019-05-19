/**
 * \file NETGeographicLib/Ellipsoid.cpp
 * \brief Implementation for NETGeographicLib::Ellipsoid class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Ellipsoid.hpp"
#include "Ellipsoid.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::Ellipsoid";

//*****************************************************************************
Ellipsoid::!Ellipsoid(void)
{
    if ( m_pEllipsoid != NULL )
    {
        delete m_pEllipsoid;
        m_pEllipsoid = NULL;
    }
}

//*****************************************************************************
Ellipsoid::Ellipsoid()
{
    try
    {
        m_pEllipsoid = new GeographicLib::Ellipsoid( GeographicLib::Ellipsoid::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
Ellipsoid::Ellipsoid(double a, double f)
{
    try
    {
        m_pEllipsoid = new GeographicLib::Ellipsoid( a, f );
    }
    catch ( std::bad_alloc err )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double Ellipsoid::ParametricLatitude(double phi)
{
    return m_pEllipsoid->ParametricLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseParametricLatitude(double beta)
{
    return m_pEllipsoid->InverseParametricLatitude( beta );
}

//*****************************************************************************
double Ellipsoid::GeocentricLatitude(double phi)
{
    return m_pEllipsoid->GeocentricLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseGeocentricLatitude(double theta)
{
    return m_pEllipsoid->InverseGeocentricLatitude( theta );
}

//*****************************************************************************
double Ellipsoid::RectifyingLatitude(double phi)
{
    return m_pEllipsoid->RectifyingLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseRectifyingLatitude(double mu)
{
    return m_pEllipsoid->InverseRectifyingLatitude( mu );
}

//*****************************************************************************
double Ellipsoid::AuthalicLatitude(double phi)
{
    return m_pEllipsoid->AuthalicLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseAuthalicLatitude(double xi)
{
    return m_pEllipsoid->InverseAuthalicLatitude( xi );
}

//*****************************************************************************
double Ellipsoid::ConformalLatitude(double phi)
{
    return m_pEllipsoid->ConformalLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseConformalLatitude(double chi)
{
    return m_pEllipsoid->InverseConformalLatitude( chi );
}

//*****************************************************************************
double Ellipsoid::IsometricLatitude(double phi)
{
    return m_pEllipsoid->IsometricLatitude( phi );
}

//*****************************************************************************
double Ellipsoid::InverseIsometricLatitude(double psi)
{
    return m_pEllipsoid->InverseIsometricLatitude( psi );
}

//*****************************************************************************
double Ellipsoid::CircleRadius(double phi)
{
    return m_pEllipsoid->CircleRadius( phi );
}

//*****************************************************************************
double Ellipsoid::CircleHeight(double phi)
{
    return m_pEllipsoid->CircleHeight( phi );
}

//*****************************************************************************
double Ellipsoid::MeridianDistance(double phi)
{
    return m_pEllipsoid->MeridianDistance( phi );
}

//*****************************************************************************
double Ellipsoid::MeridionalCurvatureRadius(double phi)
{
    return m_pEllipsoid->MeridionalCurvatureRadius( phi );
}

//*****************************************************************************
double Ellipsoid::TransverseCurvatureRadius(double phi)
{
    return m_pEllipsoid->TransverseCurvatureRadius( phi );
}

//*****************************************************************************
double Ellipsoid::NormalCurvatureRadius(double phi, double azi)
{
    return m_pEllipsoid->NormalCurvatureRadius( phi, azi );
}

//*****************************************************************************
double Ellipsoid::SecondFlatteningToFlattening(double fp)
{
    return GeographicLib::Ellipsoid::SecondFlatteningToFlattening( fp );
}

//*****************************************************************************
double Ellipsoid::FlatteningToSecondFlattening(double f)
{
    return GeographicLib::Ellipsoid::FlatteningToSecondFlattening( f );
}

//*****************************************************************************
double Ellipsoid::ThirdFlatteningToFlattening(double n)
{
    return GeographicLib::Ellipsoid::ThirdFlatteningToFlattening( n );
}

//*****************************************************************************
double Ellipsoid::FlatteningToThirdFlattening(double f)
{
    return GeographicLib::Ellipsoid::FlatteningToThirdFlattening( f );
}

//*****************************************************************************
double Ellipsoid::EccentricitySqToFlattening(double e2)
{
    return GeographicLib::Ellipsoid::EccentricitySqToFlattening( e2 );
}

//*****************************************************************************
double Ellipsoid::FlatteningToEccentricitySq(double f)
{
    return GeographicLib::Ellipsoid::FlatteningToEccentricitySq( f );
}

//*****************************************************************************
double Ellipsoid::SecondEccentricitySqToFlattening(double ep2)
{
    return GeographicLib::Ellipsoid::SecondEccentricitySqToFlattening( ep2 );
}

//*****************************************************************************
double Ellipsoid::FlatteningToSecondEccentricitySq(double f)
{
    return GeographicLib::Ellipsoid::FlatteningToSecondEccentricitySq( f );
}

//*****************************************************************************
double Ellipsoid::ThirdEccentricitySqToFlattening(double epp2)
{
    return GeographicLib::Ellipsoid::ThirdEccentricitySqToFlattening( epp2 );
}

//*****************************************************************************
double Ellipsoid::FlatteningToThirdEccentricitySq(double f)
{
    return GeographicLib::Ellipsoid::FlatteningToThirdEccentricitySq( f );
}

//*****************************************************************************
double Ellipsoid::MajorRadius::get()
{ return m_pEllipsoid->MajorRadius(); }

//*****************************************************************************
double Ellipsoid::MinorRadius::get()
{ return m_pEllipsoid->MinorRadius(); }

//*****************************************************************************
double Ellipsoid::QuarterMeridian::get()
{ return m_pEllipsoid->QuarterMeridian(); }

//*****************************************************************************
double Ellipsoid::Area::get()
{ return m_pEllipsoid->Area(); }

//*****************************************************************************
double Ellipsoid::Volume::get()
{ return m_pEllipsoid->Volume(); }

//*****************************************************************************
double Ellipsoid::Flattening::get()
{ return m_pEllipsoid->Flattening(); }

//*****************************************************************************
double Ellipsoid::SecondFlattening::get()
{ return m_pEllipsoid->SecondFlattening(); }

//*****************************************************************************
double Ellipsoid::ThirdFlattening::get()
{ return m_pEllipsoid->ThirdFlattening(); }

//*****************************************************************************
double Ellipsoid::EccentricitySq::get()
{ return m_pEllipsoid->EccentricitySq(); }

//*****************************************************************************
double Ellipsoid::SecondEccentricitySq::get()
{ return m_pEllipsoid->SecondEccentricitySq(); }

//*****************************************************************************
double Ellipsoid::ThirdEccentricitySq::get()
{ return m_pEllipsoid->ThirdEccentricitySq(); }
