/**
 * \file NETGeographicLib/GARS.cpp
 * \brief Source for NETGeographicLib::GARS class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013-2015)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GARS.hpp"
#include "GARS.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
void GARS::Forward(double lat, double lon, int prec, System::String^% gars)
{
    try
    {
        std::string l;
        GeographicLib::GARS::Forward( lat, lon, prec, l );
        gars = gcnew System::String( l.c_str() );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
void GARS::Reverse( System::String^ gars,  double% lat,  double% lon,
        int% prec, bool centerp)
{
    try
    {
        double llat, llon;
        int lprec;
        GeographicLib::GARS::Reverse( StringConvert::ManagedToUnmanaged( gars ),
            llat, llon, lprec, centerp );
        lat = llat;
        lon = llon;
        prec = lprec;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double GARS::Resolution(int prec) { return GeographicLib::GARS::Resolution(prec); }

//*****************************************************************************
int GARS::Precision(double res) { return GeographicLib::GARS::Precision(res); }
