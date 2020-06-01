#pragma ident "$Id$"



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




/* This is where all platform specific includes, defines and crud should go.
   Unless, of course, it has to go somewhere else. :-)
*/

#ifndef GPSTK_GPSTKPLATFORM_H
#define GPSTK_GPSTKPLATFORM_H

#ifdef _MSC_VER

#include <cstdlib>

#define HAVE_STRING_H 1
#define STDC_HEADERS  1

// To get rid of 'stdint.h' for Microsoft visual studio
#if (_MSC_VER < 1300)
    typedef signed char       int8_t;
    typedef signed short      int16_t;
    typedef signed int        int32_t;
    typedef unsigned char     uint8_t;
    typedef unsigned short    uint16_t;
    typedef unsigned int      uint32_t;
    typedef signed __int64    int64_t;
    typedef unsigned __int64  uint64_t;
#elif(_MSC_VER <= 1500)
    typedef signed __int8     int8_t;
    typedef signed __int16    int16_t;
    typedef signed __int32    int32_t;
    typedef unsigned __int8   uint8_t;
    typedef unsigned __int16  uint16_t;
    typedef unsigned __int32  uint32_t;
    typedef signed __int64    int64_t;
    typedef unsigned __int64  uint64_t;
#else        
    #include <stdint.h>
#endif

//#include <sys/types.h>
#include <sys/timeb.h>

#elif defined __SUNPRO_CC

#include <sys/types.h>
#include <sys/timeb.h>

#else   

#include <stdint.h>

#endif  // _MSC_VER

#endif  // GPSTK_GPSTKPLATFORM_H
