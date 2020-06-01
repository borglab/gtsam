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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008
//
//============================================================================

/**
 * @file IonexStream.hpp
 * File stream for IONEX format files
 */

#ifndef GPSTK_IONEXSTREAM_HPP
#define GPSTK_IONEXSTREAM_HPP

#include "FFTextStream.hpp"
#include "IonexHeader.hpp"

namespace gpstk
{

      /** @addtogroup IonosphereMaps */
      //@{

      /** This class provides access to IONEX files.
       *
       * @sa gpstk::IonexHeader and gpstk::IonexData for more information.
       * @sa main_ionextest.cpp for an example.
       */
   class IonexStream : public FFTextStream
   {
   public:

         /// Default constructor
      IonexStream()
            : headerRead(false) {};


         /** Common constructor.
          *
          * @param fn      IONEX file to open
          * @param mode    Mode to open \a fn.
          */
      IonexStream(const char* fn, std::ios::openmode mode=std::ios::in)
            : FFTextStream(fn, mode), headerRead(false) {};


         /// Destructor
      virtual ~IonexStream() {};


         /// Overrides open to reset the header
      virtual void open(const char* fn, std::ios::openmode mode)
      {

         FFTextStream::open(fn, mode);
         headerRead = false;
         header = IonexHeader();

      };


         /// Whether or not the IonexHeader has been read
      bool headerRead;


         /// The header for this file.
      IonexHeader header;


   }; // End of class 'IonexStream'


      //@}


}  // End of namespace gpstk
#endif   // GPSTK_IONEXSTREAM_HPP
