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
// //  Copyright 2004, The University of Texas at Austin
//
//============================================================================
/*  SVNumXRefMap.cpp
*
*   Applied Research Laboratories, The University of Texas at Austin
*
*/
   // Language Headers

   // Library Headers
   // Project Headers
#include "SVNumXRef.hpp"
#include "CivilTime.hpp"
#include "TimeString.hpp"

using namespace std;
namespace gpstk
{

SVNumXRef::SVNumXRef( )
{
   NtoBMap.insert( make_pair(  1,  I )); 
   NtoBMap.insert( make_pair(  2,  I ));
   NtoBMap.insert( make_pair(  3,  I ));
   NtoBMap.insert( make_pair(  4,  I ));
   NtoBMap.insert( make_pair(  5,  I ));
   NtoBMap.insert( make_pair(  6,  I ));
     // no NAVSTAR 07, I-7 was a launch failure
   NtoBMap.insert( make_pair(  8,  I ));
   NtoBMap.insert( make_pair(  9,  I ));
   NtoBMap.insert( make_pair( 10,  I ));
   NtoBMap.insert( make_pair( 11,  I ));
     // no NAVSTAR 12, was never launched
   NtoBMap.insert( make_pair( 13, II ));
   NtoBMap.insert( make_pair( 14, II ));
   NtoBMap.insert( make_pair( 15, II ));
   NtoBMap.insert( make_pair( 16, II ));
   NtoBMap.insert( make_pair( 17, II ));
   NtoBMap.insert( make_pair( 18, II ));
   NtoBMap.insert( make_pair( 19, II ));
   NtoBMap.insert( make_pair( 20, II ));
   NtoBMap.insert( make_pair( 21, II ));
   NtoBMap.insert( make_pair( 22,IIA ));
   NtoBMap.insert( make_pair( 23,IIA ));
   NtoBMap.insert( make_pair( 24,IIA ));
   NtoBMap.insert( make_pair( 25,IIA ));
   NtoBMap.insert( make_pair( 26,IIA ));
   NtoBMap.insert( make_pair( 27,IIA ));
   NtoBMap.insert( make_pair( 28,IIA ));
   NtoBMap.insert( make_pair( 29,IIA ));
   NtoBMap.insert( make_pair( 30,IIA ));
   NtoBMap.insert( make_pair( 31,IIA ));
   NtoBMap.insert( make_pair( 32,IIA ));
   NtoBMap.insert( make_pair( 33,IIA ));
   NtoBMap.insert( make_pair( 34,IIA ));
   NtoBMap.insert( make_pair( 35,IIA ));
   NtoBMap.insert( make_pair( 36,IIA ));
   NtoBMap.insert( make_pair( 37,IIA ));
   NtoBMap.insert( make_pair( 38,IIA ));
   NtoBMap.insert( make_pair( 39,IIA ));
   NtoBMap.insert( make_pair( 40,IIA )); 
   NtoBMap.insert( make_pair( 41,IIR ));
    // no NAVSTAR 42, IIR-1 was a launch failure
   NtoBMap.insert( make_pair( 43,IIR )); 
   NtoBMap.insert( make_pair( 44,IIR ));
   NtoBMap.insert( make_pair( 45,IIR ));
   NtoBMap.insert( make_pair( 46,IIR ));
   NtoBMap.insert( make_pair( 47,IIR ));
   NtoBMap.insert( make_pair( 48,IIR_M));
   NtoBMap.insert( make_pair( 49,IIR_M));
   NtoBMap.insert( make_pair( 50,IIR_M));
   NtoBMap.insert( make_pair( 51,IIR ));
   NtoBMap.insert( make_pair( 52,IIR_M));
   NtoBMap.insert( make_pair( 53,IIR_M));
   NtoBMap.insert( make_pair( 54,IIR ));
   NtoBMap.insert( make_pair( 55,IIR_M));
   NtoBMap.insert( make_pair( 56,IIR ));
   NtoBMap.insert( make_pair( 57,IIR_M));
   NtoBMap.insert( make_pair( 58,IIR_M));
   NtoBMap.insert( make_pair( 59,IIR ));
   NtoBMap.insert( make_pair( 60,IIR ));
   NtoBMap.insert( make_pair( 61,IIR ));
   NtoBMap.insert( make_pair( 62,IIF )); 
   NtoBMap.insert( make_pair( 63,IIF ));
   NtoBMap.insert( make_pair( 65,IIF )); 
   NtoBMap.insert( make_pair( 66,IIF )); 


      // Note: This table start with Block I values
      // Set up NAVSTAR -> PRN ID relationship
      // NAVSTAR ID first, PRN ID second
   NtoPMap.insert( std::pair<const int, XRefNode>(  1, XRefNode(  4, 
                                       CivilTime( 1978,  2, 22,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1985,  7, 17, 17, 30,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  2, XRefNode(  7, 
                                       CivilTime( 1978,  6, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1988,  2, 12, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  3, XRefNode(  6, 
                                       CivilTime( 1978, 10,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1992,  5, 18, 23, 41,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  4, XRefNode(  8, 
                                       CivilTime( 1978, 12, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1990,  5, 31, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  5, XRefNode(  5, 
                                       CivilTime( 1980,  2,  9,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1984,  5, 11, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  6, XRefNode(  9, 
                                       CivilTime( 1980,  4, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1991,  3,  6,  3, 42,  0.0, TimeSystem::GPS))));
      // no NAVSTAR 07, I-7 was a launch failure
   NtoPMap.insert( std::pair<const int, XRefNode>(  8, XRefNode( 11, 
                                       CivilTime( 1983,  7, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1993,  5,  4,  0, 20,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>(  9, XRefNode( 13, 
                                       CivilTime( 1984,  6, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1993,  5,  4, 18, 17,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 10, XRefNode( 12, 
                                       CivilTime( 1984,  9,  8,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1996,  3, 26, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 11, XRefNode(  3, 
                                       CivilTime( 1985, 10, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1994,  4, 14, 21,  0,  0.0, TimeSystem::GPS))));
      // no NAVSTAR 12, was never launched
   NtoPMap.insert( std::pair<const int, XRefNode>( 13, XRefNode(  2, 
                                       CivilTime( 1989,  6, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2004,  5, 12, 17,  1,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 14, XRefNode( 14, 
                                       CivilTime( 1989,  2, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000,  4, 14, 13, 47,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 15, XRefNode( 15, 
                                       CivilTime( 1990, 10,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007,  3, 15, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 16, XRefNode( 16, 
                                       CivilTime( 1989,  8, 18,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000, 10, 13,  0, 45,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 17, XRefNode( 17, 
                                       CivilTime( 1989, 12, 11,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2005,  2, 23, 22,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 18, XRefNode( 18, 
                                       CivilTime( 1990,  1, 24,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000,  8, 18,  7, 42,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 19, XRefNode( 19, 
                                       CivilTime( 1989, 10, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2001,  9, 11, 22,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 20, XRefNode( 20, 
                                       CivilTime( 1990,  3, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1996, 12, 13, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 21, XRefNode( 21, 
                                       CivilTime( 1990,  8,  2,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2003,  1, 27, 22,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 22, XRefNode( 22, 
                                       CivilTime( 1993,  2,  3,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2003,  8,  6, 22,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 23, XRefNode( 23, 
                                       CivilTime( 1990, 11, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2004,  2, 13, 22,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 23, XRefNode( 32, 
                                       CivilTime( 2006, 12,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 24, 
                                       CivilTime( 1991,  7,  4,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  9, 30, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 25, XRefNode( 25, 
                                       CivilTime( 1992,  2, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 26, XRefNode( 26, 
                                       CivilTime( 1992,  7,  7,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   //  NANU # (start). NANU 2012061 (end)
   //  NANU #(start). NANU 2011059 (end).   
   NtoPMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 27, 
                                       CivilTime( 1992,  9,  9,  0,  0,  0.0, TimeSystem::GPS), 								 				   CivilTime( 2011,  8, 10, 23, 59, 59.9, TimeSystem::GPS))));
   // NANU 2011105 (start). NANU 2012063 (end)     
   NtoPMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 27, 
                                       CivilTime( 2011, 12, 16, 22, 38,  0.0, TimeSystem::GPS), 								 				   CivilTime( 2012, 10,  6, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 28, XRefNode( 28, 
                                       CivilTime( 1992,  4, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1997,  8, 15, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 29, XRefNode( 29, 
                                       CivilTime( 1992, 12, 18,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007, 10, 23, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 30, XRefNode( 30, 
                                       CivilTime( 1996,  9, 12,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  8,  4, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 31, XRefNode( 31, 
                                       CivilTime( 1993,  3, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2005, 10, 24, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 32, XRefNode(  1, 
                                       CivilTime( 1992, 11, 22,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2008,  3, 17, 22,  0,  0.0, TimeSystem::GPS))));
   //  NANU 2012018 (start). NANU 2012024 (end)
   NtoPMap.insert( std::pair<const int, XRefNode>( 32, XRefNode(  24, 
                                       CivilTime( 2012,  3, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2012,  4, 24, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 33, XRefNode(  3, 
                                       CivilTime( 1996,  3, 28,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 34, XRefNode(  4, 
                                       CivilTime( 1993, 10, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 35, XRefNode(  5, 
                                       CivilTime( 1993,  8, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2009,  3, 26, 20, 31,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 35, XRefNode(  1, 
                                       CivilTime( 2009,  7, 12, 23, 59, 59.9, TimeSystem::GPS),
                                       CivilTime( 2011,  6,  1,  0,  0,  0.0, TimeSystem::GPS))));

   NtoPMap.insert( std::pair<const int, XRefNode>( 35, XRefNode(  30, 
                                       CivilTime( 2011,  8,  6, 20,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 36, XRefNode(  6, 
                                       CivilTime( 1995,  3, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 37, XRefNode(  7, 
                                       CivilTime( 1993,  5, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007,  7, 20, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 37, XRefNode(  1, 
                                       CivilTime( 2008, 10, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2009,  1,  6, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012024 (start). NANU 201249 (end)
   NtoPMap.insert( std::pair<const int, XRefNode>( 37, XRefNode(  24, 
                                       CivilTime( 2012,  4, 25,  0,  0,  0.0, TimeSystem::GPS),
									   CivilTime( 2012,  8,  7, 23, 59, 59.9, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 38, XRefNode(  8, 
                                       CivilTime( 1997, 11,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 39, XRefNode(  9, 
                                       CivilTime( 1993,  6, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 40, XRefNode( 10, 
                                       CivilTime( 1996,  7, 16,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 41, XRefNode( 14, 
                                       CivilTime( 2000, 11, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
      // no NAVSTAR 42, IIR-1 was a launch failure
   NtoPMap.insert( std::pair<const int, XRefNode>( 43, XRefNode( 13, 
                                       CivilTime( 1997,  7, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 44, XRefNode( 28,
                                       CivilTime( 2000,  7, 16,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 45, XRefNode( 21, 
                                       CivilTime( 2003,  3, 31,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 46, XRefNode( 11, 
                                       CivilTime( 1999, 10,  7,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 47, XRefNode( 22, 
                                       CivilTime( 2003, 12, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 48, XRefNode(  7, 
                                       CivilTime( 2008,  3, 15,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   //  NANU 2012003 (start). NANU 2012018 (end)
   NtoPMap.insert( std::pair<const int, XRefNode>( 49, XRefNode(  24, 
                                       CivilTime( 2012,  2,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2012,  3, 13, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012049 (start). NANU # (end)
   NtoPMap.insert( std::pair<const int, XRefNode>( 49, XRefNode( 24, 
									   CivilTime( 2012,  8,  8,  0,  0,  0.0, TimeSystem::GPS),
									   CivilTime( 2012,  8, 22, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012064 (start). NANU 2013029 (end - IMPLIED)
   NtoPMap.insert( std::pair<const int, XRefNode>( 49, XRefNode( 27, 
									   CivilTime( 2012, 10, 18,  0,  0,  0.0, TimeSystem::GPS),
									   CivilTime( 2013,  5,  7, 23, 59, 59.9, TimeSystem::GPS))));
	//  NANU 2013029 (start)
   NtoPMap.insert( std::pair<const int, XRefNode>( 49, XRefNode( 30, 
									   CivilTime( 2013,  5,  8,  0,  0,  0.0, TimeSystem::GPS),
									   CommonTime::END_OF_TIME  )));
									   
   NtoPMap.insert( std::pair<const int, XRefNode>( 49, XRefNode(  1, 
                                       CivilTime( 2009,  3, 24,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  5,  6, 16,  0,  0.0, TimeSystem::GPS))));
   NtoPMap.insert( std::pair<const int, XRefNode>( 50, XRefNode(  5, 
                                       CivilTime( 2009,  8, 27,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 51, XRefNode( 20, 
                                       CivilTime( 2000,  5, 11,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 52, XRefNode( 31, 
                                       CivilTime( 2006,  9, 25,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 53, XRefNode( 17, 
                                       CivilTime( 2005,  9, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 54, XRefNode( 18, 
                                       CivilTime( 2001,  1, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 55, XRefNode( 15, 
                                       CivilTime( 2007, 10, 17,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 56, XRefNode( 16, 
                                       CivilTime( 2003,  1, 29,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 57, XRefNode( 29, 
                                       CivilTime( 2007, 12, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 58, XRefNode( 12, 
                                       CivilTime( 2006, 11, 17,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 59, XRefNode( 19, 
                                       CivilTime( 2004,  3, 20,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 60, XRefNode( 23, 
                                       CivilTime( 2004,  6, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 61, XRefNode(  2, 
                                       CivilTime( 2004,  6,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 62, XRefNode( 25,
                                       CivilTime( 2010,  5, 28,  3,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 63, XRefNode( 1,
                                       CivilTime( 2011,  7, 20,  9, 36, 36.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   NtoPMap.insert( std::pair<const int, XRefNode>( 65, XRefNode( 24,
                                       CivilTime( 2012, 10,  4,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  ))); 
   NtoPMap.insert( std::pair<const int, XRefNode>( 66, XRefNode( 27,
                                       CivilTime( 2013,  5, 15,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  ))); 

      // Set up PRN ID -> NAVSTAR relationship
      // Note: Because of a bug in the Solaris compler version 5.x,
      // you cannot use make_pair b/c Solaris ASSUMES the key is const AND
      // Sun's implementation of pair lacks the templated copy constructor 
      // template< class a, class b> pair::pair< const pair<a,b>& p >
   PtoNMap.insert( std::pair<const int, XRefNode>(  1, XRefNode( 32, 
                                       CivilTime( 1992, 11, 22,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2008,  3, 17, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  1, XRefNode( 37, 
                                       CivilTime( 2008, 10, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2009,  1,  7, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  1, XRefNode( 49, 
                                       CivilTime( 2009,  3, 24,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  5,  6, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  1, XRefNode( 35, 
                                       CivilTime( 2011,  6,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  7, 12, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  1, XRefNode( 63, 
                                       CivilTime( 2011, 7, 20,  9, 36, 36.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  2, XRefNode( 13, 
                                       CivilTime( 1989,  6, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2004,  5, 12, 17,  1,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  2, XRefNode( 61, 
                                       CivilTime( 2004,  6,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  3, XRefNode( 11, 
                                       CivilTime( 1985, 10, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1994,  4, 14, 21,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  3, XRefNode( 33, 
                                       CivilTime( 1996,  3, 28,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  4, XRefNode(  1, 
                                       CivilTime( 1978,  2, 22,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1985,  7, 17, 17, 30,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  4, XRefNode( 34, 
                                       CivilTime( 1993, 10, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  5, XRefNode(  5, 
                                       CivilTime( 1980,  2,  9,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1984,  5, 11, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  5, XRefNode( 35, 
                                       CivilTime( 1993,  8, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2009,  3, 26, 20, 31,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  5, XRefNode( 50, 
                                       CivilTime( 2009,  8, 27,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  6, XRefNode(  3, 
                                       CivilTime( 1978, 10,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1992,  5, 18, 23, 41,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  6, XRefNode( 36, 
                                       CivilTime( 1995,  3, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  7, XRefNode(  2, 
                                       CivilTime( 1978,  6, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1988,  2, 12, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  7, XRefNode( 37, 
                                       CivilTime( 1993,  5, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007,  7, 20, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  7, XRefNode( 48, 
                                       CivilTime( 2008,  3, 15,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  8, XRefNode(  4, 
                                       CivilTime( 1978, 12, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1990,  5, 31, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  8, XRefNode( 38, 
                                       CivilTime( 1997, 11,  6,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>(  9, XRefNode(  6, 
                                       CivilTime( 1980,  4, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1991,  3,  6,  3, 42,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>(  9, XRefNode( 39, 
                                       CivilTime( 1993,  6, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 10, XRefNode( 40, 
                                       CivilTime( 1996,  7, 16,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 11, XRefNode(  8, 
                                       CivilTime( 1983,  7, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1993,  5,  4,  0, 20,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 11, XRefNode( 46, 
                                       CivilTime( 1999, 10,  7,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 12, XRefNode( 10, 
                                       CivilTime( 1984,  9,  8,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1996,  3, 26, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 12, XRefNode( 58, 
                                       CivilTime( 2006, 11, 17,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 13, XRefNode(  9, 
                                       CivilTime( 1984,  6, 13,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1993,  5,  4, 18, 17,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 13, XRefNode( 43, 
                                       CivilTime( 1997,  7, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 14, XRefNode( 14, 
                                       CivilTime( 1989,  2, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000,  4, 14, 13, 47,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 14, XRefNode( 41, 
                                       CivilTime( 2000, 11, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 15, XRefNode( 15, 
                                       CivilTime( 1990, 10,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007,  3, 15, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 15, XRefNode( 55, 
                                       CivilTime( 2007, 10, 17,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 16, XRefNode( 16, 
                                       CivilTime( 1989,  8, 18,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000, 10, 13,  0, 45,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 16, XRefNode( 56, 
                                       CivilTime( 2003,  1, 29,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 17, XRefNode( 17, 
                                       CivilTime( 1989, 12, 11,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2005,  2, 23, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 17, XRefNode( 53, 
                                       CivilTime( 2005,  9, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 18, XRefNode( 18, 
                                       CivilTime( 1990,  1, 24,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2000,  8, 18,  7, 42,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 18, XRefNode( 54, 
                                       CivilTime( 2001,  1, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 19, XRefNode( 19, 
                                       CivilTime( 1989, 10, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2001,  9, 11, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 19, XRefNode( 59, 
                                       CivilTime( 2004,  3, 20,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 20, XRefNode( 20, 
                                       CivilTime( 1990,  3, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1996, 12, 13, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 20, XRefNode( 51, 
                                       CivilTime( 2000,  5, 11,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 21, XRefNode( 21, 
                                       CivilTime( 1990,  8,  2,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2003,  1, 27, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 21, XRefNode( 45, 
                                       CivilTime( 2003,  3, 31,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 22, XRefNode( 22, 
                                       CivilTime( 1993,  2,  3,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2003,  8,  6, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 22, XRefNode( 47, 
                                       CivilTime( 2003, 12, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 23, XRefNode( 23, 
                                       CivilTime( 1990, 11, 26,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2004,  2, 13, 22,  0,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 23, XRefNode( 60, 
                                       CivilTime( 2004,  6, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 24, 
                                       CivilTime( 1991,  7,  4,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  9, 30, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012003 (start). NANU 2012018 (end).
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 49, 
                                       CivilTime( 2012,  2,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2012,  3, 13, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012049 (start). NANU #(end).   
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 49, 
							           CivilTime( 2012,  8,  8,  0,  0,  0.0, TimeSystem::GPS),
							           CivilTime( 2012,  8, 22, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012018 (start). NANU 2012024 (end).
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 32, 
                                       CivilTime( 2012,  3, 14,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2012,  4, 24, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012024 (start). NANU 2012049 (end)
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 37, 
                                       CivilTime( 2012,  4, 25,  0,  0,  0.0, TimeSystem::GPS),
     					               CivilTime( 2012,  8,  7, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012062 (start).
   PtoNMap.insert( std::pair<const int, XRefNode>( 24, XRefNode( 65, 
                                       CivilTime( 2012, 10,  4,  0,  0,  0.0, TimeSystem::GPS),
     					       	       CommonTime::END_OF_TIME)));
   PtoNMap.insert( std::pair<const int, XRefNode>( 25, XRefNode( 25,
                                       CivilTime( 1992,  2, 23,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2009, 12, 18, 22, 28,  0.0, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 25, XRefNode( 62,
                                       CivilTime( 2010,  5, 28,  3,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 26, XRefNode( 26, 
                                       CivilTime( 1992,  7,  7,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   //  NANU #(start). NANU 2011059 (end).   
   PtoNMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 27, 
                                       CivilTime( 1992,  9,  9,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  8, 10, 23, 59, 59.9, TimeSystem::GPS))));
   // NANU 2011105 (start). NANU 2012063 (end)     
   PtoNMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 27, 
                                       CivilTime( 2011, 12, 16, 22, 38,  0.0, TimeSystem::GPS),
                                       CivilTime( 2012, 10,  6, 23, 59, 59.9, TimeSystem::GPS))));
   //  NANU 2012064 (start). NANU 2013029 (end implied)
   PtoNMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 49, 
                                       CivilTime( 2012, 10, 18,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2013,  5,  7, 23, 59,  0.0, TimeSystem::GPS))));
   //
   PtoNMap.insert( std::pair<const int, XRefNode>( 27, XRefNode( 66, 
                                       CivilTime( 2013,  5, 15,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
                                       
   PtoNMap.insert( std::pair<const int, XRefNode>( 28, XRefNode( 28, 
                                       CivilTime( 1992,  4, 10,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 1997,  8, 15, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 28, XRefNode( 44,
                                       CivilTime( 2000,  7, 16,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 29, XRefNode( 29, 
                                       CivilTime( 1992, 12, 18,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2007, 10, 23, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 29, XRefNode( 57, 
                                       CivilTime( 2007, 12, 21,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 30, XRefNode( 30, 
                                       CivilTime( 1996,  9, 12,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2011,  8,  4, 23, 59, 59.9, TimeSystem::GPS))));
   //                      NANU 2013027 (end).
   PtoNMap.insert( std::pair<const int, XRefNode>( 30, XRefNode(  35, 
                                       CivilTime( 2011,  8,  5,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2013,  5,  1 , 23, 59, 59.9, TimeSystem::GPS))));
   // NANU 2013029 (start).
   PtoNMap.insert( std::pair<const int, XRefNode>( 30, XRefNode(  49, 
                                       CivilTime( 2013,  5,  8,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
                                       
   PtoNMap.insert( std::pair<const int, XRefNode>( 31, XRefNode( 31, 
                                       CivilTime( 1993,  3, 30,  0,  0,  0.0, TimeSystem::GPS),
                                       CivilTime( 2005, 10, 24, 23, 59, 59.9, TimeSystem::GPS))));
   PtoNMap.insert( std::pair<const int, XRefNode>( 31, XRefNode( 52, 
                                       CivilTime( 2006,  9, 25,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
   PtoNMap.insert( std::pair<const int, XRefNode>( 32, XRefNode( 23, 
                                       CivilTime( 2006, 12,  1,  0,  0,  0.0, TimeSystem::GPS),
                                       CommonTime::END_OF_TIME  )));
}

int SVNumXRef::getNAVSTAR( const int PRNID, const gpstk::CommonTime dt ) const
{
   SVNumXRefPair p = PtoNMap.equal_range( PRNID );
   for (SVNumXRefListCI ci=p.first; ci != p.second; ++ci )
   {
      if (ci->second.isApplicable( dt )) return( ci->second.getNAVSTARNum() );
   }
   
      // We didn't find a NAVSTAR # for this PRN ID and date, so throw an 
      // exception.
   char textOut[80];
   sprintf(textOut,"No NAVSTAR # found associated with PRN ID %d at requested date: %s.", 
            PRNID,printTime(dt,"%02m/%02d/%04Y").c_str() ); 
   std::string sout = textOut;
   NoNAVSTARNumberFound noFound( sout );
   GPSTK_THROW(noFound); 
}

bool SVNumXRef::NAVSTARIDAvailable( const int PRNID, const gpstk::CommonTime dt ) const
{
   SVNumXRefPair p = PtoNMap.equal_range( PRNID );
   for (SVNumXRefListCI ci=p.first; ci != p.second; ++ci )
   {
      if (ci->second.isApplicable( dt )) return( true );
   }
   return( false ); 
}

bool SVNumXRef::NAVSTARIDActive( const int NAVSTARID, const gpstk::CommonTime dt ) const
{
   for (SVNumXRefListCI ci=PtoNMap.begin(); ci != PtoNMap.end(); ++ci )
   {
      if (ci->second.getNAVSTARNum()==NAVSTARID &&
          ci->second.isApplicable( dt )         ) return( true );
   }
   return( false ); 
}

SVNumXRef::BlockType SVNumXRef::getBlockType( const int NAVSTARID ) const
{
   map<int,BlockType>::const_iterator i;
   i = NtoBMap.find(  NAVSTARID );
   if (i!=NtoBMap.end()) return(i->second);
   
      // We didn't find a BlockType for this NAVSTAR #, so throw an 
      // exception.
   char textOut[80];
   sprintf(textOut,"No BlockType found associated with NAVSTAR Num %d.", 
            NAVSTARID);
   std::string sout = textOut;
   NoNAVSTARNumberFound noFound( sout );
   GPSTK_THROW(noFound); 
   throw( noFound );
}

std::string SVNumXRef::getBlockTypeString( const int NAVSTARID ) const
{
   std::map<int,BlockType>::const_iterator i;
   i = NtoBMap.find( NAVSTARID );
   if (i!=NtoBMap.end())
   {
     switch( getBlockType( NAVSTARID ) )
     {
       case I: return("Block I"); break;
       case II: return("Block II"); break;
       case IIA: return("Block IIA"); break;
       case IIR: return("Block IIR"); break;
       case IIR_M: return("Block IIR_M"); break;
       case IIF: return("Block IIF"); break;
     }

   }
   return("unknown");
}

int SVNumXRef::getPRNID( const int NAVSTARID, const gpstk::CommonTime dt ) const
{
   NAVNumXRefPair p = NtoPMap.equal_range( NAVSTARID );
   // If there is only one PRNID for this SVN number return it to maintain
   // compatability with previous versions
   if( p.first == (--p.second) ) return ( p.first->second.getPRNNum() );
   ++p.second;
   for (NAVNumXRefCI ci=p.first; ci != p.second; ++ci )
   {
      if (ci->second.isApplicable( dt )) return( ci->second.getPRNNum() );
   }

      // We didn't find a PRN ID for this NAVSTAR # and date, so throw an 
      // exception.
   char textOut[80];
   sprintf(textOut,"No PRN ID found associated with NAVSTAR Num %d at requested date: %s.", 
            NAVSTARID,printTime(dt,"%02m/%02d/%04Y").c_str() ); 
   std::string sout = textOut;
   NoNAVSTARNumberFound noFound( sout );
   GPSTK_THROW(noFound); 
   throw( noFound );
}

bool SVNumXRef::PRNIDAvailable( const int NAVSTARID, const gpstk::CommonTime dt ) const
{
   NAVNumXRefPair p = NtoPMap.equal_range( NAVSTARID );
   if( p.first == (--p.second) ) return ( true );
   ++p.second;
   for (NAVNumXRefCI ci=p.first; ci != p.second; ++ci )
   {
      if (ci->second.isApplicable( dt )) return( true );
   }
   return( false ); 
}

bool SVNumXRef::BlockTypeAvailable(  const int NAVSTARID ) const
{
   map<int,BlockType>::const_iterator i;
   i = NtoBMap.find(  NAVSTARID );   
   if (i!=NtoBMap.end()) return(true);
   return(false);
}

//-------------- Methods for XRefNode -----------------
XRefNode::XRefNode( const int NumArg,
                             const gpstk::CommonTime begDT,
                             const gpstk::CommonTime endDT )
{
   Num = NumArg;
   begValid = begDT;
   endValid = endDT;
}

bool XRefNode::isApplicable( gpstk::CommonTime dt ) const
{
   if (dt>=begValid && dt<=endValid) return(true);
   return(false);
}
}
