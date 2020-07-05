/**
 * \file NETGeographicLib/MagneticCircle.cpp
 * \brief Implementation for NETGeographicLib::MagneticCircle class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/MagneticCircle.hpp"
#include "MagneticCircle.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
MagneticCircle::!MagneticCircle(void)
{
    if ( m_pMagneticCircle != NULL )
    {
        delete m_pMagneticCircle;
        m_pMagneticCircle = NULL;
    }
}

//*****************************************************************************
MagneticCircle::MagneticCircle( const GeographicLib::MagneticCircle& c )
{
    try
    {
        m_pMagneticCircle = new GeographicLib::MagneticCircle( c );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::MagneticCircle" );
    }
}

//*****************************************************************************
void MagneticCircle::Field(double lon,
    [System::Runtime::InteropServices::Out] double% Bx,
    [System::Runtime::InteropServices::Out] double% By,
    [System::Runtime::InteropServices::Out] double% Bz)
{
    double lx, ly, lz;
    m_pMagneticCircle->operator()( lon, lx, ly, lz );
    Bx = lx;
    By = ly;
    Bz = lz;
}

//*****************************************************************************
void MagneticCircle::Field(double lon,
    [System::Runtime::InteropServices::Out] double% Bx,
    [System::Runtime::InteropServices::Out] double% By,
    [System::Runtime::InteropServices::Out] double% Bz,
    [System::Runtime::InteropServices::Out] double% Bxt,
    [System::Runtime::InteropServices::Out] double% Byt,
    [System::Runtime::InteropServices::Out] double% Bzt)
{
    double lx, ly, lz, lxt, lyt, lzt;
    m_pMagneticCircle->operator()( lon, lx, ly, lz, lxt, lyt, lzt );
    Bx = lx;
    By = ly;
    Bz = lz;
    Bxt = lxt;
    Byt = lyt;
    Bzt = lzt;
}

//*****************************************************************************
double MagneticCircle::MajorRadius::get()
{
    if ( m_pMagneticCircle->Init() )
        return m_pMagneticCircle->MajorRadius();
    throw  gcnew GeographicErr("MagneticCircle::MajorRadius failed because the MagneticCircle is not initialized.");
}

//*****************************************************************************
double MagneticCircle::Flattening::get()
{
    if ( m_pMagneticCircle->Init() )
        return m_pMagneticCircle->Flattening();
    throw  gcnew GeographicErr("MagneticCircle::Flattening failed because the MagneticCircle is not initialized.");
}

//*****************************************************************************
double MagneticCircle::Latitude::get()
{
    if ( m_pMagneticCircle->Init() )
        return m_pMagneticCircle->Latitude();
    throw  gcnew GeographicErr("MagneticCircle::Latitude failed because the MagneticCircle is not initialized.");
}

//*****************************************************************************
double MagneticCircle::Height::get()
{
    if ( m_pMagneticCircle->Init() )
        return m_pMagneticCircle->Height();
    throw  gcnew GeographicErr("MagneticCircle::Height failed because the MagneticCircle is not initialized.");
}

//*****************************************************************************
double MagneticCircle::Time::get()
{
    if ( m_pMagneticCircle->Init() )
        return m_pMagneticCircle->Height();
    throw  gcnew GeographicErr("MagneticCircle::Height failed because the MagneticCircle is not initialized.");
}

//*****************************************************************************
bool MagneticCircle::Init::get() { return m_pMagneticCircle->Init(); }
