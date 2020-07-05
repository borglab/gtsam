/**
 * \file NETGeographicLib/Geoid.cpp
 * \brief Implementation for NETGeographicLib::Geoid class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/Geoid.hpp"
#include "Geoid.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
Geoid::!Geoid(void)
{
    if ( m_pGeoid != NULL )
    {
        delete m_pGeoid;
        m_pGeoid = NULL;
    }
}

//*****************************************************************************
Geoid::Geoid(System::String^ name, System::String^ path,
                bool cubic, bool threadsafe)
{
    if ( name == nullptr ) throw gcnew GeographicErr("name cannot be a null pointer.");
    if ( path == nullptr ) throw gcnew GeographicErr("path cannot be a null pointer.");
    try
    {
        m_pGeoid = new GeographicLib::Geoid(
            StringConvert::ManagedToUnmanaged( name ),
            StringConvert::ManagedToUnmanaged( path ),
            cubic, threadsafe );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::Geoid" );
    }
    catch ( const GeographicLib::GeographicErr& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}
//*****************************************************************************
void Geoid::CacheArea(double south, double west, double north, double east)
{
    try
    {
        m_pGeoid->CacheArea( south, west, north, east );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void Geoid::CacheAll()
{
    try
    {
        m_pGeoid->CacheAll();
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void Geoid::CacheClear()
{
    m_pGeoid->CacheClear();
}

//*****************************************************************************
double Geoid::Height(double lat, double lon)
{
    try
    {
        return m_pGeoid->operator()( lat, lon );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double Geoid::ConvertHeight(double lat, double lon, double h,
                            ConvertFlag d)
{
    try
    {
        return m_pGeoid->ConvertHeight( lat, lon, h,
            static_cast<GeographicLib::Geoid::convertflag>(d) );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ Geoid::DefaultGeoidPath()
{
    try
    {
        return StringConvert::UnmanagedToManaged(
            GeographicLib::Geoid::DefaultGeoidPath() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ Geoid::DefaultGeoidName()
{
    try
    {
        return StringConvert::UnmanagedToManaged(
            GeographicLib::Geoid::DefaultGeoidName() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ Geoid::Description::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->Description()); }

//*****************************************************************************
System::String^ Geoid::DateTime::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->DateTime()); }

//*****************************************************************************
System::String^ Geoid::GeoidFile::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->GeoidFile()); }

//*****************************************************************************
System::String^ Geoid::GeoidName::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->GeoidName()); }

//*****************************************************************************
System::String^ Geoid::GeoidDirectory::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->GeoidDirectory()); }

//*****************************************************************************
System::String^ Geoid::Interpolation::get()
{ return StringConvert::UnmanagedToManaged(m_pGeoid->Interpolation()); }

//*****************************************************************************
double Geoid::MaxError::get() { return m_pGeoid->MaxError(); }

//*****************************************************************************
double Geoid::RMSError::get() { return m_pGeoid->RMSError(); }

//*****************************************************************************
double Geoid::Offset::get() { return m_pGeoid->Offset(); }

//*****************************************************************************
double Geoid::Scale::get() { return m_pGeoid->Scale(); }

//*****************************************************************************
bool Geoid::ThreadSafe::get() { return m_pGeoid->ThreadSafe(); }

//*****************************************************************************
bool Geoid::Cache::get() { return m_pGeoid->Cache(); }

//*****************************************************************************
double Geoid::CacheWest::get() { return m_pGeoid->CacheWest(); }

//*****************************************************************************
double Geoid::CacheEast::get() { return m_pGeoid->CacheEast(); }

//*****************************************************************************
double Geoid::CacheNorth::get() { return m_pGeoid->CacheNorth(); }

//*****************************************************************************
double Geoid::CacheSouth::get() { return m_pGeoid->CacheSouth(); }

//*****************************************************************************
double Geoid::MajorRadius::get() { return m_pGeoid->MajorRadius(); }

//*****************************************************************************
double Geoid::Flattening::get() { return m_pGeoid->Flattening(); }
