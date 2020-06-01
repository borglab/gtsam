/// @file HelmertTransform.cpp

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

#include <ostream>
#include <iomanip>

#include "HelmertTransform.hpp"
#include "TimeString.hpp"
#include "YDSTime.hpp"
#include "geometry.hpp"

using namespace gpstk;
using namespace std;

namespace gpstk {

   // Explicit constructor, from the 7 parameters.
   HelmertTransform::HelmertTransform(
                    const ReferenceFrame& from, const ReferenceFrame& to,
                    const double& Rx, const double& Ry, const double& Rz,
                    const double& Tx, const double& Ty, const double& Tz,
                    const double& Sc, const string& Desc, CommonTime epoch)
      throw(InvalidRequest)
   {
      // copy input
      // NB input is in degrees, members in radians
      rx = Rx*DEG_TO_RAD; ry = Ry*DEG_TO_RAD; rz = Rz*DEG_TO_RAD;
      tx = Tx; ty = Ty; tz = Tz;
      Scale = Sc;
      description = Desc;
      Epoch = epoch;
      fromFrame = from;
      toFrame = to;
      if(from == ReferenceFrame::Unknown || to == ReferenceFrame::Unknown) {
         InvalidRequest e("Invalid Helmert transformation with Unknown frame");
         GPSTK_THROW(e);
      }

      // check that rotation angles are small; sin x ~ x at 0.244 radians = 13.9 deg
      if(::fabs(rx) > 1.e-3 || ::fabs(ry) > 1.e-3 || ::fabs(rz) > 1.e-3) {
         InvalidRequest e("Invalid Helmert transformation : "
                                 "small angle approximation.");
         GPSTK_THROW(e);
      }

      // rotation matrix. NB. small angle approximation is used. NB. by construction
      // transpose(Rotation) == inverse(Rotation) (given small angle approximation).
      Rotation = Matrix<double>(3,3,0.0);
      Rotation(0,0) = 1.0;
      Rotation(0,1) = -rz;
      Rotation(0,2) = ry;

      Rotation(1,0) = rz;
      Rotation(1,1) = 1.0;
      Rotation(1,2) = -rx;

      Rotation(2,0) = -ry;
      Rotation(2,1) = rx;
      Rotation(2,2) = 1.0;

      // translation vector
      Translation = Vector<double>(3);
      Translation(0) = tx;
      Translation(1) = ty;
      Translation(2) = tz;
   }

   // Dump the object to a multi-line string including reference frames, the
   // 7 parameters and description.
   string HelmertTransform::asString() const throw()
   {
      ostringstream oss;
      oss << "Helmert Transformation"
          << " from " << fromFrame.asString()
          << " to " << toFrame.asString() + ":\n"
          << scientific << setprecision(4)
          << "  Scale factor : " << Scale
          << fixed << " = " << Scale/PPB << " ppb" << endl
          << "  Rotation angles (deg):"
          << scientific
          << "  X : " << rx*RAD_TO_DEG 
          << ",  Y : " << ry*RAD_TO_DEG 
          << ",  Z : " << rz*RAD_TO_DEG << endl
          << "  Rotation angles (mas):"
          << fixed
          << "  X : " << rx*RAD_TO_DEG/DEG_PER_MAS
          << ",  Y : " << ry*RAD_TO_DEG/DEG_PER_MAS
          << ",  Z : " << rz*RAD_TO_DEG/DEG_PER_MAS << endl
          << "  Translation (meters):"
          << "  X : " << tx
          << ",  Y : " << ty
          << ",  Z : " << tz << endl
          << "  Beginning Epoch: "
          << (Epoch == CommonTime::BEGINNING_OF_TIME ? string(" [all times]")
               : printTime(Epoch,"%Y/%02m/%02d %2H:%02M:%06.3f = %F %.3g %P")) << endl
          << "  Description: " << description;
      return (oss.str());
   }

   // Transform Position to another frame using this transform or its inverse.
   // @param Position& pos position to be transformed; unchanged on output.
   // @param Position& result position after transformation.
   // @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
   void HelmertTransform::transform(const Position& pos, Position& result)
      throw(InvalidRequest)
   {
      if(pos.getReferenceFrame() == fromFrame) {           // transform
         result = pos;
         result.transformTo(Position::Cartesian);
         Vector<double> vec(3),res(3);
         vec(0) = result[0];
         vec(1) = result[1];
         vec(2) = result[2];
         res = Rotation*vec + Scale*vec + Translation;
         result[0] = res(0);
         result[1] = res(1);
         result[2] = res(2);
         result.setReferenceFrame(toFrame);
      }
      else if(pos.getReferenceFrame() == toFrame) {        // inverse transform
         result = pos;
         result.transformTo(Position::Cartesian);
         Vector<double> vec(3),res(3);
         vec(0) = result[0];
         vec(1) = result[1];
         vec(2) = result[2];
         res = transpose(Rotation) * (vec - Scale*vec - Translation);
         result[0] = res(0);
         result[1] = res(1);
         result[2] = res(2);
         result.setReferenceFrame(fromFrame);
      }
      else {
         InvalidRequest e("Helmert tranformation cannot act on frame "
                           + pos.getReferenceFrame().asString());
         GPSTK_THROW(e);
      }
   }

   // time of PZ90 change
   const CommonTime HelmertTransform::PZ90Epoch(
      YDSTime(2007,263,61200.0,TimeSystem::UTC));
   //HelmertTransform::PZ90Epoch(2454364L,61200L,0.0,TimeSystem::UTC);

   // array of pre-defined HelmertTransforms
   const HelmertTransform HelmertTransform::stdTransforms[stdCount] =
   {
      HelmertTransform(ReferenceFrame::WGS84, ReferenceFrame::ITRF,
           0, 0, 0,  0.0, 0.0, 0.0,  0,
           string("WGS84 to ITRF identity transform, a default value\n        "
                  "(\"...since 1997, the WGS84 GPS broadcast ...\n         "
                  "is consistent with the ITRS at better than 5-cm level.\"\n       "
                  "Boucher & Altamimi 2001)"),
           YDSTime(1997,1,0.0,TimeSystem::UTC)),
           //CommonTime(2450450L,0L,0.0,TimeSystem::UTC)),

      // PZ90 WGS84
      HelmertTransform(ReferenceFrame::PZ90, ReferenceFrame::WGS84,
           -19*DEG_PER_MAS, -4*DEG_PER_MAS, 353*DEG_PER_MAS,
           0.07, 0.0, -0.77,
           -3*PPB,
           string("PZ90 to WGS84, determined by IGEX-98, reference\n       "
                  "\"ITRS, PZ-90 and WGS 84: current realizations\n       "
                  "and the related transformation parameters,\"\n       "
                  "Journal Geodesy (2001), 75:613, by Boucher and Altamimi.\n       "
                  "Use before 20 Sept 2007 17:00 UTC (ICD-2008 v5.1 table 3.2).")),

      HelmertTransform(ReferenceFrame::PZ90, ReferenceFrame::WGS84,
           0, 0, 0,  -0.36, 0.08, 0.18,  0,
           string("PZ90.02 to ITRF2000, from Sergey Revnivykh, GLONASS PNT\n       "
                  "Information Analysis Center, 47th CGSIC Meeting and ION\n       "
                  "GNSS 2007, Fort Worth, Texas, implemented by GLONASS\n       "
                  "20 Sept 2007 17:00 UTC (ICD-2008 v5.1 table 3.2)."),
           PZ90Epoch),

      // PZ90 ITRF
      HelmertTransform(ReferenceFrame::PZ90, ReferenceFrame::ITRF,
           -19*DEG_PER_MAS, -4*DEG_PER_MAS, 353*DEG_PER_MAS,
           0.07, 0.0, -0.77,
           -3*PPB,
           string("PZ90 to ITRF(WGS84), determined by IGEX-98, reference\n       "
                  "\"ITRS, PZ-90 and WGS 84: current realizations\n       "
                  "and the related transformation parameters,\"\n       "
                  "Journal Geodesy (2001), 75:613, by Boucher and Altamimi.\n       "
                  "Use before 20 Sept 2007 17:00 UTC (ICD-2008 v5.1 table 3.2).")),

      HelmertTransform(ReferenceFrame::PZ90, ReferenceFrame::ITRF,
           0, 0, 0,  -0.36, 0.08, 0.18,  0,
           string("PZ90.02 to ITRF2000, from Sergey Revnivykh, GLONASS PNT\n       "
                  "Information Analysis Center, 47th CGSIC Meeting and ION\n       "
                  "GNSS 2007, Fort Worth, Texas, implemented by GLONASS\n       "
                  "20 Sept 2007 17:00 UTC (ICD-2008 v5.1 table 3.2)."),
           PZ90Epoch),

      // add more transforms here, and increase HelmertTransform::stdCount in .cpp
   };

} // end namespace gpstk
