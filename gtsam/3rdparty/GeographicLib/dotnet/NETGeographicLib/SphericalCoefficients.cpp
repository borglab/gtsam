/**
 * \file NETGeographicLib/SphericalCoefficients.cpp
 * \brief Implementation for NETGeographicLib::SphericalCoefficients class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/SphericalEngine.hpp"
#include "SphericalCoefficients.h"

using namespace NETGeographicLib;

//*****************************************************************************
SphericalCoefficients::SphericalCoefficients(const GeographicLib::SphericalEngine::coeff& c)
{
    m_N = c.N();
    m_nmx = c.nmx();
    m_mmx = c.mmx();
    int csize = Csize( c.nmx(), c.mmx() );
    int ssize = Ssize( c.nmx(), c.mmx() );
    int offset = csize - ssize;
    m_C = gcnew array<double>( csize );
    m_S = gcnew array<double>( ssize );
    for ( int i = 0; i < csize; i++ ) m_C[i] = c.Cv(i);
    for ( int i = 0; i < ssize; i++ ) m_S[i] = c.Sv(i+offset);
}
