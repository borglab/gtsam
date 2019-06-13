/**
 * \file NETGeographicLib/MGRS.cpp
 * \brief Implementation for NETGeographicLib::MGRS class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/MGRS.hpp"
#include "MGRS.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
void MGRS::Forward(int zone, bool northp, double x, double y,
                int prec,
                [System::Runtime::InteropServices::Out] System::String^% mgrs)
{
    try
    {
        std::string lmgrs;
        GeographicLib::MGRS::Forward( zone, northp, x, y, prec, lmgrs );
        mgrs = gcnew System::String( lmgrs.c_str() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void MGRS::Forward(int zone, bool northp, double x, double y, double lat,
                int prec, System::String^% mgrs)
{
    try
    {
        std::string lmgrs;
        GeographicLib::MGRS::Forward( zone, northp, x, y, lat, prec, lmgrs );
        mgrs = gcnew System::String( lmgrs.c_str() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void MGRS::Reverse(System::String^ mgrs,
                [System::Runtime::InteropServices::Out] int% zone,
                [System::Runtime::InteropServices::Out] bool% northp,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] int% prec,
                bool centerp )
{
    try
    {
        double lx, ly;
        int lzone, lprec;
        bool lnorthp;
        GeographicLib::MGRS::Reverse( StringConvert::ManagedToUnmanaged( mgrs ),
            lzone, lnorthp, lx, ly, lprec, centerp );
        x = lx;
        y = ly;
        zone = lzone;
        prec = lprec;
        northp = lnorthp;
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double MGRS::MajorRadius() { return GeographicLib::UTMUPS::MajorRadius(); }

//*****************************************************************************
double MGRS::Flattening() { return GeographicLib::UTMUPS::Flattening(); }
