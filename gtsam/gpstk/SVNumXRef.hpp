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
/*   SVNumXRef.hpp
*
*   Applied Research Laboratories, The University of Texas at Austin
*   June 2007
*
*   This class implements a cross-reference between PRN numbers and NAVSTAR IDs.
*   Since PRN numbers are re-used (historically, there's a limit of 1-32 in
*   the PRN IDs) it's necessary provide a several->one PRN ID ->NAVSTAR mapping.
*   Originally, the NAVSTAR->PRN ID mapping is unique. However, this is no longer
*   true and several -> one mapping now necessary in both directions.
*
*   As a result, the cross reference methods support the ability to provide a
*   date so the method can select between multiple options.
*
*   Note: Thie information is not contained anywhere in the navigation message.
*   Therefore, it needs to come from "outside the system".  The information
*   contained in the class constructor will also need to updated with each
*   new launch and disposal.  This is a sore point, but there appears to be
*   no easy way around it.
*
*   Note: The data in the module is derived from several sources and there
*   are some issues of interpretation.
*    - The USNO website ftp://tycho.usno.navy.mil/pub/gps/gpsb2.txt contains
*      a very helpful history
*    - This class only attempts to track the relationship between PRN ID and
*      NAVSTAR numbers.  There's no guarantee a SV was operationally, broadcasting,
*      or healthy.
*
*  MEMBER METHODS
*     SVNumXRef( ); - Constructor.  Works from a fixed table that is compiled
*                     into the code.
*     int getNAVSTAR( const int PRNID, const gpstk::CommonTime dt ) const; -
*                     Given a PRNID and a date, return the NAVSTAR number
*                     related to thsat PRNID at the date provided.  May
*                     throw "NoNAVSTARNumFound" error. The date defaults to
*                     the current date if one is not provided.
*     int getPRNID( const int NAVSTARID, const gpstk::CommonTime dt ) const; -
*                     Given a NAVSTAR Number and a date, return the corresponding
*                     PRNID related to the NAVSTAR Number on the specified date.
*                     May throw "No PRNNumberFound" error." The date defaults to
*                     the current date if one is not provided.
*
*  The following four members parallel the preceding four and provide a
*  means of determining is the specified information is available prior to
*  making a "get" call.  This is useful if you want to avoid writing
*  "try/catch" blocks.
*     bool PRNIDavailable( const int NAVSTARID, const gpstk::CommonTime dt ) const;
*     bool NAVSTARIDAvailable( const int PRNID, const gpstk::CommonTime dt ) const;
*
*
*     bool NAVSTARIDActive( const int NAVSTARID, const gpstk::CommonTime dt ) const; -
*     Given a NAVSTAR ID and a date, return true if the specified NAVSTAR ID was
*     active on the date provided. The date defaults to the current date if one
*     is not provided.
*
*/
#ifndef SVNUMXREF_HPP
#define SVNUMXREF_HPP

   // Language Headers
#include <map>
#include <utility>

   // Library Headers
#include "CommonTime.hpp"
#include "SystemTime.hpp"
#include "gps_constants.hpp"
#include "Exception.hpp"

   // Project Headers

namespace gpstk
{

NEW_EXCEPTION_CLASS( NoPRNNumberFound, gpstk::Exception);
NEW_EXCEPTION_CLASS( NoNAVSTARNumberFound, gpstk::Exception);


class XRefNode
{
   public:
      XRefNode( const int NumArg,
                     const gpstk::CommonTime begDT,
                     const gpstk::CommonTime endDT );
      int getNAVSTARNum() const;
      int getPRNNum() const;
      gpstk::CommonTime getBeginTime() const;
      gpstk::CommonTime getEndTime() const;
      bool isApplicable( gpstk::CommonTime dt ) const;

   protected:
      int Num;
      gpstk::CommonTime begValid;
      gpstk::CommonTime endValid;
};

typedef std::multimap<int, XRefNode>::const_iterator SVNumXRefListCI;
typedef std::pair<SVNumXRefListCI,SVNumXRefListCI> SVNumXRefPair;
typedef std::multimap<int, XRefNode>::const_iterator NAVNumXRefCI;
typedef std::pair<NAVNumXRefCI,NAVNumXRefCI> NAVNumXRefPair;

class SVNumXRef
{
   public:

      enum BlockType
      {
      I,
      II,
      IIA,
      IIR,
      IIR_M,
      IIF
      };
      SVNumXRef( );
      ~SVNumXRef() {}
      BlockType getBlockType( const int NAVSTARID ) const;
      std::string getBlockTypeString( const int NAVSTARID ) const;
      int getNAVSTAR( const int PRNID, const gpstk::CommonTime dt = SystemTime() ) const;
      int getPRNID( const int NAVSTARID, const gpstk::CommonTime dt = SystemTime() ) const;
      bool PRNIDAvailable( const int NAVSTARID, const gpstk::CommonTime dt = SystemTime() ) const;
      bool NAVSTARIDAvailable( const int PRNID, const gpstk::CommonTime dt = SystemTime() ) const;
      bool BlockTypeAvailable( const int NAVSTARID ) const;
      bool NAVSTARIDActive( const int NAVSTARID, const gpstk::CommonTime dt = SystemTime() ) const;

   protected:
      std::multimap<int,XRefNode> NtoPMap;
      std::multimap<int,XRefNode> PtoNMap;
      std::map<int,BlockType> NtoBMap;
};

inline int XRefNode::getNAVSTARNum() const { return(Num); }
inline int XRefNode::getPRNNum() const { return(Num); }
inline gpstk::CommonTime XRefNode::getBeginTime() const { return( begValid ); }
inline gpstk::CommonTime XRefNode::getEndTime() const { return( endValid ); }


}
#endif
