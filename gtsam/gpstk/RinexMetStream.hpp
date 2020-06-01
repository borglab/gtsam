#pragma ident "$Id$"

/**
 * @file RinexMetStream.hpp
 * File stream for RINEX 2 & 3 Met files.
 */

#ifndef GPSTK_RINEXMETSTREAM_HPP
#define GPSTK_RINEXMETSTREAM_HPP

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

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

#include "FFTextStream.hpp"
#include "RinexMetHeader.hpp"

namespace gpstk
{
  /** @addtogroup RinexMet */
  //@{

  /**
   * This class performs file i/o on a RINEX MET file for the
   * RinexMetHeader and RinexMetData classes.
   *
   * @sa rinex_met_read_write.cpp for an example.
   * @sa rinex_met_test.cpp for an example.
   * @sa RinexMetData.
   * @sa RinexMetHeader for information on writing RINEX met files.
   *
   * @warning When writing RinexMetData, the internal
   * RinexMetStream::headerData must have the correct observation
   * types set for what you want to write out.  If you don't set any,
   * no data will be written. See RinexMetHeader for more information
   * on this.
   */
  class RinexMetStream : public FFTextStream
  {

  public:

    /// default constructor
    RinexMetStream()
      : headerRead(false)
    {}

    /** Constructor 
     * Opens a file named \a fn using ios::openmode \a mode.
     */
    RinexMetStream(const char* fn, std::ios::openmode mode=std::ios::in)
      : FFTextStream(fn, mode), headerRead(false) {};

    /// Destructor
    virtual ~RinexMetStream() {}

    /// overrides open to reset the header
    virtual void open(const char* fn, std::ios::openmode mode)
    {
      FFTextStream::open(fn, mode);
      headerRead = false; 
      header = RinexMetHeader();
    }

    /// RINEX met header for this file.
    RinexMetHeader header;

    /// Flag showing whether or not the header has been read.
    bool headerRead;

  };

  //@}

}

#endif
