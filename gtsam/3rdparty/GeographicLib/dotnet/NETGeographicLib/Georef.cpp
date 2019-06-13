/**
 * \file NETGeographicLib/Georef.cpp
 * \brief Source for NETGeographicLib::Georef class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013-2015)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Georef.hpp"
#include "Georef.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
void Georef::Forward(double lat, double lon, int prec, System::String^% georef)
{
    try
    {
        std::string l;
        GeographicLib::Georef::Forward( lat, lon, prec, l );
        georef = gcnew System::String( l.c_str() );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
void Georef::Reverse( System::String^ georef,  double% lat,  double% lon,
        int% prec, bool centerp)
{
    try
    {
        double llat, llon;
        int lprec;
        GeographicLib::Georef::Reverse( StringConvert::ManagedToUnmanaged( georef ),
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
double Georef::Resolution(int prec) { return GeographicLib::Georef::Resolution(prec); }

//*****************************************************************************
int Georef::Precision(double res) { return GeographicLib::Georef::Precision(res); }
