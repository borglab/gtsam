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

//  PCodeConst.hpp - Constants for GPS X-register manipulation



#ifndef PCodeConst_HPP
#define PCodeConst_HPP

//#define PCODE_DEBUG

namespace gpstk
{
   /** @addtogroup code */
   /**
    *  PCodeConst.hpp maintains a variety of "magic numbers" related
    *  to P-code generation and used throughout the P-code generator.
    */
   enum codeType { P_CODE, Y_CODE, BOTH }; 
    
      /// Number of bits assumed to be in a unsigned long int
   const int MAX_BIT = 32;
   
      /// Maximum PRN Code number (1-n) 
   const int MAX_PRN_CODE = 210;
 
      /// Number of X1 epochs in one day
   const int X1_PER_DAY = 57600;

      /// Number of 4 byte unsigned ints necessary to hold 6 sec of P-code
   const long NUM_6SEC_WORDS = 1918125;
   
      /// Number of 4 byte unsigned ints necessary to hold an X2 sequence (with leading delay)
   const long NUM_X2_WORDS   = 1918131;
   
      /// INIT variables are starting conditions of 12-bit registers (IS-GPS-200)
   const unsigned int X1A_INIT = 0x0248;
   const unsigned int X1B_INIT = 0x0554;
   const unsigned int X2A_INIT = 0x0925;
   const unsigned int X2B_INIT = 0x0554;

      /// TAPS variables denote which stages of 12-bit registers are XOR'd.   
   const unsigned int X1A_TAPS = 0x0CA0;
   const unsigned int X1B_TAPS = 0x0F93;
   const unsigned int X2A_TAPS = 0x0FDD;
   const unsigned int X2B_TAPS = 0x098E;

      /// X?_MAX_EPOCH is the maximum number of epochs in a sequence
   const int XA_MAX_EPOCH = 3750;
   const int XB_MAX_EPOCH = 3749;
   
      /// X?_COUNT is the number of bits in an epoch
   const int XA_COUNT = 4092;
   const int XB_COUNT = 4093;
   
      /** XA_EPOCH_DELAY and XB_EPOCH_DELAY allow for precession of X1B and X2B wrt
       * to X1A and X2A at the end of each X1A epoch and X2A epoch.  End
       * of week delays are handled elsewhere.
       */
   const long XA_EPOCH_DELAY  =   0;
   const long XB_EPOCH_DELAY  = 343;
   
      /// The 37 chip delay at the end of every X2A epoch 
   const long X2A_EPOCH_DELAY = 37;
   //@}
} // namespace

#endif
