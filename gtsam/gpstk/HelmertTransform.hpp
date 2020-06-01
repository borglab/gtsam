/// @file HelmertTransform.hpp

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

#ifndef GPSTK_HELMERT_TRANSFORM_HPP
#define GPSTK_HELMERT_TRANSFORM_HPP

#include <map>
#include <string>

#include "Exception.hpp"
#include "ReferenceFrame.hpp"
#include "CommonTime.hpp"
#include "Position.hpp"
#include "Vector.hpp"
#include "Matrix.hpp"
#include "Xvt.hpp"

namespace gpstk
{
   /// Helmert transformations, which are 7-parameter transformations between
   /// reference frames (i.e. ECEF position coordinates). A Helmert tranformation is
   /// defined by 7 parameters: a rotation(3), a translation(3) and scale factor(1).
   class HelmertTransform
   {
   public:

      /// Default constructor
      HelmertTransform() throw() : fromFrame(ReferenceFrame::Unknown),
                                   toFrame(ReferenceFrame::Unknown),
                                   description("Undefined")
         {};

      /// Explicit constructor, from the 7 parameters.
      /// All the inputs are unchanged.
      /// @param ReferenceFrame& from Transform takes "from" -> "to"
      /// @param ReferenceFrame& to Transform takes "from" -> "to"
      /// @param double& Rx X axis rotation angle in degrees
      /// @param double& Ry Y axis rotation angle in degrees
      /// @param double& Rz Z axis rotation angle in degrees
      /// @param double& Tx X axis translation in meters
      /// @param double& Ty Y axis translation in meters
      /// @param double& Tz Z axis translation in meters
      /// @param double& Scale scale factor (dimensionless)
      /// @param std::string& Desc description of the transform, should include
      /// @param CommonTime& epoch time when transform became applicable (default=BOT)
      /// reference frames and an indication of the source (e.g. literature citation).
      /// @throw if the transform is invalid.
      HelmertTransform(const ReferenceFrame& from, const ReferenceFrame& to,
                       const double& Rx, const double& Ry, const double& Rz,
                       const double& Tx, const double& Ty, const double& Tz,
                       const double& Scale, const std::string& Desc,
                       CommonTime epoch=CommonTime::BEGINNING_OF_TIME)
         throw(InvalidRequest);

      /// Dump the object to a multi-line string including reference frames, the
      /// 7 parameters and description.
      std::string asString() const throw();

      /// Transform Position to another frame using this transform or its inverse.
      /// @param Position& pos position to be transformed; unchanged on output.
      /// @param Position& result position after transformation.
      /// @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
      void transform(const Position& pos, Position& result) throw(InvalidRequest);

      /// Transform a 3-vector, in the given frame, using this Helmert transformation.
      /// @param Vector<double>& vec 3-vector of position coordinates in "from" frame.
      /// @param ReferenceFrame& frame Transform takes "frame" -> "new-frame"
      /// @param Vector<double>& result 3-vector in "new-frame" frame.
      /// @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
      void transform(const Vector<double>& vec, const ReferenceFrame& frame,
                     Vector<double>& result)
         throw(InvalidRequest)
      {
         if(vec.size() > 3) {
            InvalidRequest e("Input Vector is not of length 3");
            GPSTK_THROW(e);
         }
         try {
            Position pos(vec[0],vec[1],vec[2],Position::Cartesian), res;
            pos.setReferenceFrame(frame);
            transform(pos, res);
            result = Vector<double>(3);
            result[0] = res.X();
            result[1] = res.Y();
            result[2] = res.Z();
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }

      /// Transform a Triple using this Helmert transformation or its inverse.
      /// @param Triple& vec containing position and frame to be transformed.
      /// @param ReferenceFrame& frame Transform takes "frame" -> "new-frame"
      /// @param Triple& result with position in new frame
      /// @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
      void transform(const Triple& vec, const ReferenceFrame& frame, Triple& result)
         throw(InvalidRequest)
      {
         try {
            Position pos(vec, Position::Cartesian), res;
            pos.setReferenceFrame(frame);
            transform(pos, res);
            result[0] = res[0];
            result[1] = res[1];
            result[2] = res[2];
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }

      /// Transform an Xvt using this Helmert transformation or its inverse.
      /// @param Xvt& vec containing position and frame to be transformed.
      /// @param Xvt& result with position in new frame
      /// @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
      void transform(const Xvt& xvt, Xvt& result)
         throw(InvalidRequest)
      {
         try {
            Position pos(xvt.x, Position::Cartesian), res;
            pos.setReferenceFrame(xvt.frame);
            transform(pos, res);
            result = xvt;
            result.x[0] = res[0];
            result.x[1] = res[1];
            result.x[2] = res[2];
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }

      /// Transform 3 doubles, in the given frame, using this Helmert transformation.
      /// @param double& x,y,z 3-vector of position coordinates in "from" frame.
      /// @param ReferenceFrame& frame Transform takes "frame" -> "new-frame"
      /// @param double& rx,ry,rz result 3-vector in "new-frame" frame.
      /// @throw if transformation, or inverse, cannot act on ReferenceFrame of input.
      void transform(const double& x, const double& y, const double& z,
                     const ReferenceFrame& frame,
                     double& rx, double& ry, double& rz)
         throw(InvalidRequest)
      {
         try {
            Position pos(x,y,z,Position::Cartesian), res;
            pos.setReferenceFrame(frame);
            transform(pos, res);
            rx = res.X();
            ry = res.Y();
            rz = res.Z();
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }

      // accessors
      ReferenceFrame getFromFrame(void) const throw()
      { return fromFrame; }

      ReferenceFrame getToFrame(void) const throw()
      { return toFrame; }

      CommonTime getEpoch(void) const throw()
      { return Epoch; }

      /// Epoch at which GLONASS transitions from PZ90 to PZ90.02
      static const CommonTime PZ90Epoch;

      /// length of array of pre-defined HelmertTransforms
      static const int stdCount = 5; // equal to number listed in HelmertTransform.cpp

      /// array of all pre-defined HelmertTransforms
      static const HelmertTransform stdTransforms[stdCount];

   protected:
      // member data

      // this transformation takes fromFrame -> toFrame
      ReferenceFrame fromFrame;  ///< Reference frame to which *this is applied.
      ReferenceFrame toFrame;    ///< Reference frame resulting from *this transform.

      // the 7 parameters that define the tranformation
      double rx;                 ///< X axis rotation in radians
      double ry;                 ///< Y axis rotation in radians
      double rz;                 ///< Z axis rotation in radians
      double tx;                 ///< X axis translation in meters
      double ty;                 ///< Y axis translation in meters
      double tz;                 ///< Z axis translation in meters
      double Scale;              ///< scale factor, dimensionless, 0 means no scale

      // transform quantities derived from the 7 parameters
      Matrix<double> Rotation;   ///< the transform 3x3 rotation matrix (w/o scale)
      Vector<double> Translation;///< the transform 3-vector in meters

      /// epoch at which transform is first applicable
      CommonTime Epoch;

      /// an arbitrary string describing the transform; it should include the source.
      std::string description;

   }; // end class HelmertTransform

   /// degrees per milliarcsecond (1e-3/3600.)
   static const double DEG_PER_MAS = 2.77777777777e-7;

   /// radians per milliarcsecond
   static const double RAD_PER_MAS = 4.84813681e-9;

   /// parts per billion
   static const double PPB = 1.e-9;

} // end namespace gpstk

#endif
