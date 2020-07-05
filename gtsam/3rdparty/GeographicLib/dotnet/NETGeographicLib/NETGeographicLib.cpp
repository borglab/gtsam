/**
 * \file NETGeographicLib/NETGeographicLib.cpp
 * \brief Implementation for NETGeographicLib Utility functions.
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Config.h"
#include "GeographicLib/Utility.hpp"
#include "NETGeographicLib.h"

using namespace System::Runtime::InteropServices;
using namespace NETGeographicLib;

//*****************************************************************************
std::string StringConvert::ManagedToUnmanaged( System::String^ s )
{
    System::IntPtr buffer = Marshal::StringToHGlobalAnsi(s);
    std::string output( reinterpret_cast<const char*>(buffer.ToPointer()) );
    Marshal::FreeHGlobal(buffer);
    return output;
}

//*****************************************************************************
System::String^ VersionInfo::GetString()
{
    return gcnew System::String(GEOGRAPHICLIB_VERSION_STRING);
}

//*****************************************************************************
int VersionInfo::MajorVersion()
{
    return GEOGRAPHICLIB_VERSION_MAJOR;
}

//*****************************************************************************
int VersionInfo::MinorVersion()
{
    return GEOGRAPHICLIB_VERSION_MINOR;
}

//*****************************************************************************
int VersionInfo::Patch()
{
    return GEOGRAPHICLIB_VERSION_PATCH;
}

//*****************************************************************************
double Utility::FractionalYear( System::String^ s )
{
    return GeographicLib::Utility::fractionalyear<double>(
        StringConvert::ManagedToUnmanaged( s ) );
}
