/**
 * \file NETGeographicLib/MagneticModel.cpp
 * \brief Implementation for NETGeographicLib::MagneticModel class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/MagneticModel.hpp"
#include "MagneticModel.h"
#include "GeographicLib/MagneticCircle.hpp"
#include "MagneticCircle.h"
#include "Geocentric.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

const char BADALLOC[] = "Failed to allocate memory for a GeographicLib::MagneticModel";

//*****************************************************************************
MagneticModel::!MagneticModel(void)
{
    if ( m_pMagneticModel != NULL )
    {
        delete m_pMagneticModel;
        m_pMagneticModel = NULL;
    }
}

//*****************************************************************************
MagneticModel::MagneticModel(System::String^ name,
                System::String^ path,
                Geocentric^ earth)
{
    if ( name == nullptr ) throw gcnew GeographicErr("name cannot be a null pointer.");
    if ( path == nullptr ) throw gcnew GeographicErr("path cannot be a null pointer.");
    if ( earth == nullptr ) throw gcnew GeographicErr("earth cannot be a null pointer.");

    try
    {
        const GeographicLib::Geocentric* pGeocentric =
            reinterpret_cast<const GeographicLib::Geocentric*>(
                earth->GetUnmanaged()->ToPointer() );

        m_pMagneticModel = new GeographicLib::MagneticModel(
            StringConvert::ManagedToUnmanaged( name ),
            StringConvert::ManagedToUnmanaged( path ),
            *pGeocentric );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch (const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
MagneticModel::MagneticModel(System::String^ name,
                System::String^ path)
{
    if ( name == nullptr ) throw gcnew GeographicErr("name cannot be a null pointer.");
    if ( path == nullptr ) throw gcnew GeographicErr("path cannot be a null pointer.");

    try
    {
        m_pMagneticModel = new GeographicLib::MagneticModel(
            StringConvert::ManagedToUnmanaged( name ),
            StringConvert::ManagedToUnmanaged( path ),
            GeographicLib::Geocentric::WGS84() );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr( BADALLOC );
    }
    catch (const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
void MagneticModel::Field(double t, double lat, double lon, double h,
                [System::Runtime::InteropServices::Out] double% Bx,
                [System::Runtime::InteropServices::Out] double% By,
                [System::Runtime::InteropServices::Out] double% Bz)
{
    double lx, ly, lz;
    m_pMagneticModel->operator()( t, lat, lon, h, lx, ly, lz );
    Bx = lx;
    By = ly;
    Bz = lz;
}

//*****************************************************************************
void MagneticModel::Field(double t, double lat, double lon, double h,
                [System::Runtime::InteropServices::Out] double% Bx,
                [System::Runtime::InteropServices::Out] double% By,
                [System::Runtime::InteropServices::Out] double% Bz,
                [System::Runtime::InteropServices::Out] double% Bxt,
                [System::Runtime::InteropServices::Out] double% Byt,
                [System::Runtime::InteropServices::Out] double% Bzt)
{
    double lx, ly, lz, lxt, lyt, lzt;
    m_pMagneticModel->operator()( t, lat, lon, h, lx, ly, lz, lxt, lyt, lzt );
    Bx = lx;
    By = ly;
    Bz = lz;
    Bxt = lxt;
    Byt = lyt;
    Bzt = lzt;
}

//*****************************************************************************
MagneticCircle^ MagneticModel::Circle(double t, double lat, double h)
{
    try
    {
        return gcnew MagneticCircle( m_pMagneticModel->Circle( t, lat, h ) );
    }
    catch ( std::bad_alloc )
    {
        throw gcnew GeographicErr("Failed to allocate memory for a MagneticCircle in MagneticModel::Circle");
    }
}

//*****************************************************************************
void MagneticModel::FieldComponents(double Bx, double By, double Bz,
                            [System::Runtime::InteropServices::Out] double% H,
                            [System::Runtime::InteropServices::Out] double% F,
                            [System::Runtime::InteropServices::Out] double% D,
                            [System::Runtime::InteropServices::Out] double% I)
{
    double lh, lf, ld, li;
    GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, lh, lf, ld, li);
    H = lh;
    F = lf;
    D = ld;
    I = li;
}

//*****************************************************************************
void MagneticModel::FieldComponents(double Bx, double By, double Bz,
                            double Bxt, double Byt, double Bzt,
                    [System::Runtime::InteropServices::Out] double% H,
                    [System::Runtime::InteropServices::Out] double% F,
                    [System::Runtime::InteropServices::Out] double% D,
                    [System::Runtime::InteropServices::Out] double% I,
                    [System::Runtime::InteropServices::Out] double% Ht,
                    [System::Runtime::InteropServices::Out] double% Ft,
                    [System::Runtime::InteropServices::Out] double% Dt,
                    [System::Runtime::InteropServices::Out] double% It)
{
    double lh, lf, ld, li, lht, lft, ldt, lit;
    GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, Bxt, Byt, Bzt,
        lh, lf, ld, li, lht, lft, ldt, lit);
    H = lh;
    F = lf;
    D = ld;
    I = li;
    Ht = lht;
    Ft = lft;
    Dt = ldt;
    It = lit;
}

//*****************************************************************************
System::String^ MagneticModel::Description::get()
{
    return StringConvert::UnmanagedToManaged( m_pMagneticModel->Description() );
}

//*****************************************************************************
System::String^ MagneticModel::DateTime::get()
{
    return StringConvert::UnmanagedToManaged( m_pMagneticModel->DateTime() );
}

//*****************************************************************************
System::String^ MagneticModel::MagneticFile::get()
{
    return StringConvert::UnmanagedToManaged( m_pMagneticModel->MagneticFile() );
}

//*****************************************************************************
System::String^ MagneticModel::MagneticModelName::get()
{
    return StringConvert::UnmanagedToManaged( m_pMagneticModel->MagneticModelName() );
}

//*****************************************************************************
System::String^ MagneticModel::MagneticModelDirectory::get()
{
    return StringConvert::UnmanagedToManaged( m_pMagneticModel->MagneticModelDirectory() );
}

//*****************************************************************************
System::String^ MagneticModel::DefaultMagneticPath()
{
    return StringConvert::UnmanagedToManaged( GeographicLib::MagneticModel::DefaultMagneticPath() );
}

//*****************************************************************************
System::String^ MagneticModel::DefaultMagneticName()
{
    return StringConvert::UnmanagedToManaged( GeographicLib::MagneticModel::DefaultMagneticName() );
}

//*****************************************************************************
double MagneticModel::MinHeight::get()
{ return m_pMagneticModel->MinHeight(); }

//*****************************************************************************
double MagneticModel::MaxHeight::get()
{ return m_pMagneticModel->MaxHeight(); }

//*****************************************************************************
double MagneticModel::MinTime::get() { return m_pMagneticModel->MinTime(); }

//*****************************************************************************
double MagneticModel::MaxTime::get() { return m_pMagneticModel->MaxTime(); }

//*****************************************************************************
double MagneticModel::MajorRadius::get()
{ return m_pMagneticModel->MajorRadius(); }

//*****************************************************************************
double MagneticModel::Flattening::get()
{ return m_pMagneticModel->Flattening(); }
