/**
 * \file NETGeographicLib/DMS.cpp
 * \brief Implementation for NETGeographicLib::DMS class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/DMS.hpp"
#include "DMS.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
double DMS::Decode( System::String^ dms,
    [System::Runtime::InteropServices::Out] Flag% ind)
{
    try
    {
        GeographicLib::DMS::flag lind;
        double out =  GeographicLib::DMS::Decode( StringConvert::ManagedToUnmanaged(dms), lind );
        ind = static_cast<Flag>(lind);
        return out;
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
void DMS::DecodeLatLon(System::String^ dmsa, System::String^ dmsb,
                        [System::Runtime::InteropServices::Out] double% lat,
                        [System::Runtime::InteropServices::Out] double% lon,
                        bool longfirst )
{
    try
    {
        double llat, llon;
        GeographicLib::DMS::DecodeLatLon( StringConvert::ManagedToUnmanaged( dmsa ),
            StringConvert::ManagedToUnmanaged( dmsb ), llat, llon, longfirst );
        lat = llat;
        lon = llon;
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
double DMS::DecodeAngle(System::String^ angstr)
{
    try
    {
        return GeographicLib::DMS::DecodeAngle(StringConvert::ManagedToUnmanaged( angstr ));
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
double DMS::DecodeAzimuth(System::String^ azistr)
{
    try
    {
        return GeographicLib::DMS::DecodeAzimuth(StringConvert::ManagedToUnmanaged( azistr ));
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}

//*****************************************************************************
System::String^ DMS::Encode(double angle, Component trailing, unsigned prec,
                            Flag ind, char dmssep)
{
    try
    {
        return StringConvert::UnmanagedToManaged(
            GeographicLib::DMS::Encode( angle,
                static_cast<GeographicLib::DMS::component>(trailing),
                prec,
                static_cast<GeographicLib::DMS::flag>(ind),
                dmssep )
            );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}
//*****************************************************************************
System::String^ DMS::Encode(double angle, unsigned prec, Flag ind,
                            char dmssep )
{
    try
    {
        return StringConvert::UnmanagedToManaged(
            GeographicLib::DMS::Encode( angle,
                prec,
                static_cast<GeographicLib::DMS::flag>(ind),
                dmssep )
            );
    }
    catch ( const std::exception& xcpt )
    {
        throw gcnew GeographicErr( xcpt.what() );
    }
}
