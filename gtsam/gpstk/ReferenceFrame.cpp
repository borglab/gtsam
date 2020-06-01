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
//  Copyright 2009, The University of Texas at Austin
//
//============================================================================

#include "ReferenceFrame.hpp"

using namespace std;

namespace gpstk
{
   // Static initialization of const std::strings for asString().
   // Must parallel enum Frames in ReferenceFrames.hpp.
   // NB: DO NOT use std::map here; on some systems initialization fails.
   const string ReferenceFrame::Strings[count] =
     {
       string("Unknown"),
       string("WGS84"),          // WGS84, assumed to be the latest version
       string("WGS84(G730)"),    // WGS84, GPS week 730 version
       string("WGS84(G873)"),    // WGS84, GPS week 873 version
       string("WGS84(G1150)"),   // WGS84, GPS week 1150 version
       string("ITRF"),           // ITRF, assumed to be the latest version
       string("PZ90"),           // PZ90 (GLONASS) assumed to be the latest version
       string("PZ90KGS")         // PZ90 (KGS) the "original"
     };

   ReferenceFrame::ReferenceFrame(const string str) throw()
   {
      frame = Unknown;
      for(int i=0; i<count; i++) {
         if(Strings[i] == str) {
            frame = static_cast<Frames>(i);
            return;
         }
      }
   }

   void ReferenceFrame::setReferenceFrame(const Frames& frm)
      throw()
   {
      if(frm < 0 || frm >= count)
         frame = Unknown;
      else
         frame = frm;
   }

   ostream& operator<<(ostream os, const ReferenceFrame& rf)
   {
      return os << rf.asString();
   }
}   // end namespace
