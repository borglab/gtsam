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

#include "gpstkplatform.h"
#include "SystemTime.hpp"
#include "ANSITime.hpp"
//#include <sys/time.h> // #include <ctime>      // for gettimeofday()

namespace gpstk
{
   SystemTime& SystemTime::update()
   {
#if defined(ANSI_ONLY)
      time_t t;
      time( &t );
      *this = ANSITime( t );
#elif defined(WIN32)
      _timeb t;
      _ftime( &t );
      tv.tv_sec = t.time;
      tv.tv_usec = t.millitm * 1000;
#else
      gettimeofday( &tv, NULL );
#endif

      return *this;
   }

} // namespace
