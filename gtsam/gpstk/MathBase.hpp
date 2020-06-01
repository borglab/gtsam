/// @file MathBase.hpp
/// Basic math #defines (ABS, SQRT, etc)
 
//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

#ifndef GPSTK_MATH_BASE_HPP
#define GPSTK_MATH_BASE_HPP

//@fix MSVC doesnt like std::sqrt or std::abs, and disabling
//extensions allows abs(double) to be used instead of fabs()
#ifdef _MSC_VER
#undef _MSC_EXTENSIONS
#endif
#include <cmath>
#ifdef _MSC_VER
#define _MSC_EXTENSIONS
#endif

namespace gpstk
{
// do Doxygen elsewhere
#ifdef _MSC_VER
#define ABS(x)  ::abs(x)
#define SQRT(x) ::sqrt(x)
#define MAX(x,y) std::max(x,y)
#else
#define ABS(x)  std::abs(x)
#define SQRT(x) std::sqrt(x)
#define MAX(x,y) std::max(x,y)
#endif

}  // namespace gpstk
#endif
