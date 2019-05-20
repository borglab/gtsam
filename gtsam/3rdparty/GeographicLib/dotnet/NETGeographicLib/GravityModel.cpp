/**
 * \file NETGeographicLib/GravityModel.cpp
 * \brief Implementation for NETGeographicLib::GravityModel class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "stdafx.h"
#include "GeographicLib/GravityModel.hpp"
#include "GravityModel.h"
#include "GeographicLib/GravityCircle.hpp"
#include "GravityCircle.h"
#include "NormalGravity.h"
#include "NETGeographicLib.h"

using namespace NETGeographicLib;

//*****************************************************************************
GravityModel::!GravityModel(void)
{
    if ( m_pGravityModel != NULL )
    {
        delete m_pGravityModel;
        m_pGravityModel = NULL;
    }
}

//*****************************************************************************
GravityModel::GravityModel(System::String^ name, System::String^ path)
{
    if ( name == nullptr ) throw gcnew GeographicErr("name cannot be a null pointer.");
    if ( path == nullptr ) throw gcnew GeographicErr("path cannot be a null pointer.");
    try
    {
        m_pGravityModel = new GeographicLib::GravityModel(
            StringConvert::ManagedToUnmanaged( name ),
            StringConvert::ManagedToUnmanaged( path ));
    }
    catch ( std::bad_alloc err )
    {
        throw gcnew GeographicErr( "Failed to allocate memory for a GeographicLib::GravityModel" );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
double GravityModel::Gravity(double lat, double lon, double h,
                    [System::Runtime::InteropServices::Out] double% gx,
                    [System::Runtime::InteropServices::Out] double% gy,
                    [System::Runtime::InteropServices::Out] double% gz)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->Gravity( lat, lon, h, lgx, lgy, lgz );
    gx = lgx;
    gy = lgy;
    gz = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::Disturbance(double lat, double lon, double h,
        [System::Runtime::InteropServices::Out] double% deltax,
        [System::Runtime::InteropServices::Out] double% deltay,
        [System::Runtime::InteropServices::Out] double% deltaz)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->Disturbance( lat, lon, h, lgx, lgy, lgz );
    deltax = lgx;
    deltay = lgy;
    deltaz = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::GeoidHeight(double lat, double lon)
{
    return m_pGravityModel->GeoidHeight( lat, lon );
}

//*****************************************************************************
void GravityModel::SphericalAnomaly(double lat, double lon, double h,
        [System::Runtime::InteropServices::Out] double% Dg01,
        [System::Runtime::InteropServices::Out] double% xi,
        [System::Runtime::InteropServices::Out] double% eta)
{
    double lgx, lgy, lgz;
    m_pGravityModel->SphericalAnomaly( lat, lon, h, lgx, lgy, lgz );
    Dg01 = lgx;
    xi = lgy;
    eta = lgz;
}

//*****************************************************************************
double GravityModel::W(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% gX,
                [System::Runtime::InteropServices::Out] double% gY,
                [System::Runtime::InteropServices::Out] double% gZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->W( X, Y, Z, lgx, lgy, lgz );
    gX = lgx;
    gY = lgy;
    gZ = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::V(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% GX,
                [System::Runtime::InteropServices::Out] double% GY,
                [System::Runtime::InteropServices::Out] double% GZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->V( X, Y, Z, lgx, lgy, lgz );
    GX = lgx;
    GY = lgy;
    GZ = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::T(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% deltaX,
                [System::Runtime::InteropServices::Out] double% deltaY,
                [System::Runtime::InteropServices::Out] double% deltaZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->T( X, Y, Z, lgx, lgy, lgz );
    deltaX = lgx;
    deltaY = lgy;
    deltaZ = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::T(double X, double Y, double Z)
{
    return m_pGravityModel->T( X, Y, Z );
}

//*****************************************************************************
double GravityModel::U(double X, double Y, double Z,
                [System::Runtime::InteropServices::Out] double% gammaX,
                [System::Runtime::InteropServices::Out] double% gammaY,
                [System::Runtime::InteropServices::Out] double% gammaZ)
{
    double lgx, lgy, lgz;
    double out = m_pGravityModel->U( X, Y, Z, lgx, lgy, lgz );
    gammaX = lgx;
    gammaY = lgy;
    gammaZ = lgz;
    return out;
}

//*****************************************************************************
double GravityModel::Phi(double X, double Y,
    [System::Runtime::InteropServices::Out] double% fX,
    [System::Runtime::InteropServices::Out] double% fY)
{
    double lgx, lgy;
    double out = m_pGravityModel->Phi( X, Y, lgx, lgy );
    fX = lgx;
    fY = lgy;
    return out;
}

//*****************************************************************************
GravityCircle^ GravityModel::Circle(double lat, double h, Mask caps )
{
    try
    {
        return gcnew GravityCircle( m_pGravityModel->Circle( lat, h, (unsigned)caps ) );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ GravityModel::Description::get()
{
    return StringConvert::UnmanagedToManaged( m_pGravityModel->Description() );
}

//*****************************************************************************
System::String^ GravityModel::DateTime::get()
{
    return StringConvert::UnmanagedToManaged( m_pGravityModel->DateTime() );
}

//*****************************************************************************
System::String^ GravityModel::GravityFile::get()
{
    return StringConvert::UnmanagedToManaged( m_pGravityModel->GravityFile() );
}

//*****************************************************************************
System::String^ GravityModel::GravityModelName::get()
{
    return StringConvert::UnmanagedToManaged( m_pGravityModel->GravityModelName() );
}

//*****************************************************************************
System::String^ GravityModel::GravityModelDirectory::get()
{
    return StringConvert::UnmanagedToManaged( m_pGravityModel->GravityModelDirectory() );
}

//*****************************************************************************
System::String^ GravityModel::DefaultGravityPath()
{
    try
    {
        return StringConvert::UnmanagedToManaged( GeographicLib::GravityModel::DefaultGravityPath() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
System::String^ GravityModel::DefaultGravityName()
{
    try
    {
        return StringConvert::UnmanagedToManaged( GeographicLib::GravityModel::DefaultGravityName() );
    }
    catch ( const std::exception& err )
    {
        throw gcnew GeographicErr( err.what() );
    }
}

//*****************************************************************************
NormalGravity^ GravityModel::ReferenceEllipsoid()
{
    return gcnew NormalGravity( m_pGravityModel->ReferenceEllipsoid() );
}

//*****************************************************************************
double GravityModel::MajorRadius::get()
{ return m_pGravityModel->MajorRadius(); }

//*****************************************************************************
double GravityModel::MassConstant::get()
{ return m_pGravityModel->MassConstant(); }

//*****************************************************************************
double GravityModel::ReferenceMassConstant::get()
{ return m_pGravityModel->ReferenceMassConstant(); }

//*****************************************************************************
double GravityModel::AngularVelocity::get()
{ return m_pGravityModel->AngularVelocity(); }

//*****************************************************************************
double GravityModel::Flattening::get()
{ return m_pGravityModel->Flattening(); }
