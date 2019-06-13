/**
 * \file NETGeographicLib/EllipticFunction.cpp
 * \brief Implementation for NETGeographicLib::EllipticFunction class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/EllipticFunction.hpp"
#include "EllipticFunction.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for the GeographicLib::EllipticFunction.";

//*****************************************************************************
EllipticFunction::EllipticFunction(double k2, double alpha2)
{
    try
    {
        m_pEllipticFunction = new GeographicLib::EllipticFunction( k2, alpha2 );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
EllipticFunction::EllipticFunction(double k2, double alpha2, double kp2, double alphap2)
{
    try
    {
        m_pEllipticFunction = new GeographicLib::EllipticFunction( k2, alpha2, kp2, alphap2 );
    }
    catch ( std::bad_alloc err )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
}

//*****************************************************************************
EllipticFunction::!EllipticFunction()
{
    if ( m_pEllipticFunction != NULL )
    {
        delete m_pEllipticFunction;
        m_pEllipticFunction = NULL;
    }
}

//*****************************************************************************
void EllipticFunction::Reset(double k2, double alpha2 )
{
    m_pEllipticFunction->Reset( k2, alpha2 );
}

//*****************************************************************************
void EllipticFunction::Reset(double k2, double alpha2, double kp2, double alphap2)
{
    m_pEllipticFunction->Reset( k2, alpha2, kp2, alphap2 );
}

//*****************************************************************************
double EllipticFunction::K()
{
    return m_pEllipticFunction->K();
}

//*****************************************************************************
double EllipticFunction::E()
{
    return m_pEllipticFunction->E();
}

//*****************************************************************************
double EllipticFunction::D()
{
    return m_pEllipticFunction->D();
}

//*****************************************************************************
double EllipticFunction::KE()
{
    return m_pEllipticFunction->KE();
}

//*****************************************************************************
double EllipticFunction::Pi()
{
    return m_pEllipticFunction->Pi();
}

//*****************************************************************************
double EllipticFunction::G()
{
    return m_pEllipticFunction->G();
}

//*****************************************************************************
double EllipticFunction::H()
{
    return m_pEllipticFunction->H();
}

//*****************************************************************************
double EllipticFunction::F(double phi)
{
    return m_pEllipticFunction->F( phi );
}

//*****************************************************************************
double EllipticFunction::E(double phi)
{
    return m_pEllipticFunction->E( phi );
}

//*****************************************************************************
double EllipticFunction::Ed(double ang)
{
    return m_pEllipticFunction->Ed(ang);
}

//*****************************************************************************
double EllipticFunction::Einv(double x)
{
    return m_pEllipticFunction->Einv(x);
}

//*****************************************************************************
double EllipticFunction::Pi(double phi)
{
    return m_pEllipticFunction->Pi(phi);
}

//*****************************************************************************
double EllipticFunction::D(double phi)
{
    return m_pEllipticFunction->D(phi);
}

//*****************************************************************************
double EllipticFunction::G(double phi)
{
    return m_pEllipticFunction->G(phi);
}

//*****************************************************************************
double EllipticFunction::H(double phi)
{
    return m_pEllipticFunction->H(phi);
}

//*****************************************************************************
double EllipticFunction::F(double sn, double cn, double dn)
{
    return m_pEllipticFunction->F( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::E(double sn, double cn, double dn)
{
    return m_pEllipticFunction->E( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::Pi(double sn, double cn, double dn)
{
    return m_pEllipticFunction->Pi( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::D(double sn, double cn, double dn)
{
    return m_pEllipticFunction->D( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::G(double sn, double cn, double dn)
{
    return m_pEllipticFunction->G( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::H(double sn, double cn, double dn)
{
    return m_pEllipticFunction->H( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaF(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaF( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaE(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaE( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaEinv(double stau, double ctau)
{
    return m_pEllipticFunction->deltaEinv( stau, ctau );
}

//*****************************************************************************
double EllipticFunction::deltaPi(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaPi( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaD(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaD( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaG(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaG( sn, cn, dn );
}

//*****************************************************************************
double EllipticFunction::deltaH(double sn, double cn, double dn)
{
    return m_pEllipticFunction->deltaH( sn, cn, dn );
}

//*****************************************************************************
void EllipticFunction::sncndn(double x,
    [System::Runtime::InteropServices::Out] double% sn,
    [System::Runtime::InteropServices::Out] double% cn,
    [System::Runtime::InteropServices::Out] double% dn)
{
    double lsn, lcn, ldn;
    m_pEllipticFunction->sncndn( x, lsn, lcn, ldn );
    sn = lsn;
    cn = lcn;
    dn = ldn;
}

//*****************************************************************************
double EllipticFunction::Delta(double sn, double cn)
{
    return m_pEllipticFunction->Delta( sn, cn );
}

//*****************************************************************************
double EllipticFunction::RF(double x, double y, double z)
{
    return GeographicLib::EllipticFunction::RF( x, y, z );
}

//*****************************************************************************
double EllipticFunction::RF(double x, double y)
{
    return GeographicLib::EllipticFunction::RF( x, y );
}

//*****************************************************************************
double EllipticFunction::RC(double x, double y)
{
    return GeographicLib::EllipticFunction::RC( x, y );
}

//*****************************************************************************
double EllipticFunction::RG(double x, double y, double z)
{
    return GeographicLib::EllipticFunction::RG( x, y, z );
}

//*****************************************************************************
double EllipticFunction::RG(double x, double y)
{
    return GeographicLib::EllipticFunction::RG( x, y );
}

//*****************************************************************************
double EllipticFunction::RJ(double x, double y, double z, double p)
{
    return GeographicLib::EllipticFunction::RJ( x, y, z, p );
}

//*****************************************************************************
double EllipticFunction::RD(double x, double y, double z)
{
    return GeographicLib::EllipticFunction::RD( x, y, z );
}

//*****************************************************************************
double EllipticFunction::k2::get()
{ return m_pEllipticFunction->k2(); }

//*****************************************************************************
double EllipticFunction::kp2::get()
{ return m_pEllipticFunction->kp2(); }

//*****************************************************************************
double EllipticFunction::alpha2::get()
{ return m_pEllipticFunction->alpha2(); }

//*****************************************************************************
double EllipticFunction::alphap2::get()
{ return m_pEllipticFunction->alpha2(); }
