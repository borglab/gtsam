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

/**
 * @file PackedNavBits.cpp
 * Engineering units navigation message abstraction.
 */
#include <math.h>
#include <iostream>
#include <iomanip>

#include "PackedNavBits.hpp"
#include "GPSWeekSecond.hpp"
#include "TimeString.hpp"
//#include "CivilTime.hpp"
//#include "YDSTime.hpp"
#include "GNSSconstants.hpp"

namespace gpstk
{
   using namespace std;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
   PackedNavBits::PackedNavBits()
                 :bits(900),
                  bits_used(0),
                  transmitTime(CommonTime::BEGINNING_OF_TIME)
   {
      transmitTime.setTimeSystem(TimeSystem::GPS);
   }
#pragma clang diagnostic pop
   PackedNavBits::PackedNavBits(const SatID& satSysArg, 
                                const ObsID& obsIDArg,
                                const CommonTime& transmitTimeArg)
                                :bits(900),
                                 bits_used(0)
   {
      satSys = satSysArg;
      obsID = obsIDArg;
      transmitTime = transmitTimeArg;
   }

      // Copy constructor
   PackedNavBits::PackedNavBits(const PackedNavBits& right)
   {
      satSys = right.satSys; 
      obsID = right.obsID;
      transmitTime = right.transmitTime;
      bits_used = right.bits_used;
      bits.resize(bits_used);
      for (int i=0;i<bits_used;i++)
      {
         bits[i] = right.bits[i];
      }
   }
 /*  
      // Copy assignment
   PackedNavBits& PackedNavBits::operator=(const PackedNavBits& right)
   {
      satSys = right.satSys; 
      obsID = right.obsID;
      transmitTime = right.transmitTime;
      bits_used = right.bits_used;
      bits.resize(bits_used);
      for (int i=0;i<bits_used;i++)
      {
         bits[i] = right.bits[i];
      }
   }
*/

   PackedNavBits* PackedNavBits::clone() const
   {
      return new PackedNavBits (*this); 
   }

   void PackedNavBits::setSatID(const SatID& satSysArg)
   {
      satSys = satSysArg;
      return;
   }

   void PackedNavBits::setObsID(const ObsID& obsIDArg)
   {
      obsID = obsIDArg;
      return;
   }

   void PackedNavBits::setTime(const CommonTime& TransmitTimeArg)
   {
      transmitTime = TransmitTimeArg;
      return;
   }
   
   void PackedNavBits::clearBits()
   {
      bits.clear();
      bits_used = 0;
   }

   ObsID PackedNavBits::getobsID() const
   {
      return(obsID);
   }

   SatID PackedNavBits::getsatSys() const
   {
      return(satSys);
   }

   CommonTime PackedNavBits::getTransmitTime() const
   {
      return(transmitTime);
   }

   size_t PackedNavBits::getNumBits() const
   {
      return(bits_used);
   }

         /***    UNPACKING FUNCTIONS *********************************/
   uint64_t PackedNavBits::asUint64_t(const int startBit, 
                                      const int numBits ) const
      throw(InvalidParameter)                                    
   {
      uint64_t temp = 0L;       // Set up a temporary variable with a known size
                                // It needs to be AT LEAST 33 bits.
      size_t stop = startBit + numBits;
      if (stop>bits.size())
      {
         InvalidParameter exc("Requested bits not present.");
         GPSTK_THROW(exc);
      }
      for (size_t i=startBit; i<stop; ++i)
      {
         temp <<= 1;
         if (bits[i]) temp++;
      }
      return( temp ); 
   }

   unsigned long PackedNavBits::asUnsignedLong(const int startBit, 
                                               const int numBits, 
                                               const int scale ) const
   {
      uint64_t temp = asUint64_t( startBit, numBits );
      unsigned long ulong = (unsigned long) temp;
      ulong *= scale; 
      return( ulong ); 
   }

   long PackedNavBits::asLong(const int startBit, const int numBits,   
                              const int scale) const
   {
      int64_t s = SignExtend( startBit, numBits);
      return( (long) (s * scale ) );
   }

   double PackedNavBits::asUnsignedDouble(const int startBit, const int numBits, 
                                          const int power2) const
   {
      uint64_t uint = asUint64_t( startBit, numBits );
      
         // Convert to double and scale
      double dval = (double) uint;
      dval *= pow(static_cast<double>(2), power2);
      return( dval );
   }

   double PackedNavBits::asSignedDouble(const int startBit, const int numBits,  
                                        const int power2 ) const
   {
      int64_t s = SignExtend( startBit, numBits);

         // Convert to double and scale
      double dval = (double) s;
      dval *= pow(static_cast<double>(2), power2);
      return( dval );
   }

   double PackedNavBits::asDoubleSemiCircles(const int startBits, const int numBits, 
                                             const int power2) const
   {
      double drad = asSignedDouble( startBits, numBits, power2);
      return (drad*PI);
   };

   std::string PackedNavBits::asString(const int startBit, const int numChars) const 
   {
      int CHAR_SIZE = 8;
      string out = " ";
      int currentStart = startBit;
      for (int i = 0; i < numChars; ++i)
      {
         uint64_t temp = asUint64_t(currentStart, CHAR_SIZE);
         char ch = (char) temp;
         out += ch;
         currentStart += CHAR_SIZE;
      }
      return(out);
   }

         /***    PACKING FUNCTIONS *********************************/
   void PackedNavBits::addUnsignedLong( const unsigned long value, 
                                        const int numBits,
                                        const int scale ) 
      throw(InvalidParameter)
   {
      uint64_t out = (uint64_t) value;
      out /= scale;

      uint64_t test = pow(static_cast<double>(2),numBits) - 1; 
      if ( out > test )
      {
         InvalidParameter exc("Scaled value too large for specifed bit length");
         GPSTK_THROW(exc);
      }
      addUint64_t( out, numBits ); 
   }  

   void PackedNavBits::addLong( const long value, const int numBits, const int scale ) 
      throw(InvalidParameter)
   {
      union
      {
         uint64_t u_out;
         int64_t out;
      };
      out = (int64_t) value;
      out /= scale;

      int64_t test = pow(static_cast<double>(2),numBits-1) - 1; 
      if ( ( out > test ) || ( out < -( test + 1 ) ) )
      {
         InvalidParameter exc("Scaled value too large for specifed bit length");
         GPSTK_THROW(exc);
      }
      addUint64_t( u_out, numBits ); 
   } 

   void PackedNavBits::addUnsignedDouble( const double value, const int numBits,
                                          const int power2 ) 
      throw(InvalidParameter)
   {
      uint64_t out = (uint64_t) ScaleValue(value, power2);
      uint64_t test = pow(static_cast<double>(2),numBits) - 1;
      if ( out > test )
      {
         InvalidParameter exc("Scaled value too large for specifed bit length");
         GPSTK_THROW(exc);
      }

      addUint64_t( out, numBits ); 
   } 

   void PackedNavBits::addSignedDouble( const double value, const int numBits,
                                        const int power2 ) 
      throw(InvalidParameter)
   {
      union
      {
         uint64_t u_out;
         int64_t out;
      };
      out = (int64_t) ScaleValue(value, power2);
      int64_t test = pow(static_cast<double>(2),numBits-1) - 1; 
      if ( ( out > test ) || ( out < -( test + 1 ) ) )
      {
         InvalidParameter exc("Scaled value too large for specifed bit length");
         GPSTK_THROW(exc);
      }
      addUint64_t( u_out, numBits ); 
   }

   void PackedNavBits::addDoubleSemiCircles( const double Radians, const int numBits, 
                                             const int power2)
      throw(InvalidParameter)
   {
      union
      {
         uint64_t u_out;
         int64_t out;
      };
      double temp = Radians/PI;
      out = (int64_t) ScaleValue(temp, power2);
      int64_t test = pow(static_cast<double>(2), numBits-1) - 1; 
      if ( ( out > test ) || ( out < -( test + 1 ) ) )
      {
         InvalidParameter exc("Scaled value too large for specifed bit length");
         GPSTK_THROW(exc);
      }
      addUint64_t( u_out, numBits );
   }

   void PackedNavBits::addString( const string String, const int numChars ) 
      throw(InvalidParameter)
   {
      int numPadBlanks = 0;
      int numToCopy = 0;
      if (numChars < int(String.length()))
      {
         numPadBlanks = 0;
         numToCopy = numChars;
      }
      else if (numChars > int(String.length()))
      {
         numToCopy = String.length();
         numPadBlanks = numChars - numToCopy;
      }
      else
      {
         numToCopy = numChars;
      }
      int i;
      for (i = 0; i < numToCopy; ++i)
      {
         char ch = String[i];
         bool valid = false;
         if ( ('A' <= ch && ch <= 'Z') || ('0' <= ch && ch <= ':') || (' ' == ch) ||
            ('"' == ch) || ('\'' == ch) || ('+' == ch) || ('-' <= ch && ch <= '/') ||
            (0xF8 == ch) ) valid = true;
         if (!valid)
         {
            InvalidParameter exc("Invalid character '<< ch <<' in text string. ");
            GPSTK_THROW(exc);
         }
         uint64_t out = (uint64_t) ch;
         addUint64_t(out, 8);
      }
      uint64_t space = 0x00000020;
      for (i = 0; i < numPadBlanks; ++i)
         addUint64_t(space, 8);
   }  

   void PackedNavBits::addUint64_t( const uint64_t value, const int numBits )
   {
      size_t ndx = bits_used;
      uint64_t mask = 0x0000000000000001L;
      mask <<= (numBits-1);
      for (int i=0; i<numBits; ++i)
      {
         bits[ndx] = false;
         if (value & mask) 
         {
             //set the bits to true
            bits[ndx] = true;
         }
         mask>>= 1;
         ndx++;
      }
      bits_used += numBits;
   }

   void PackedNavBits::trimsize()
   {
      bits.resize(bits_used);
   }

   int64_t PackedNavBits::SignExtend( const int startBit, const int numBits) const
   {
      union
      {
         uint64_t u;
         int64_t s;
      };
      u = asUint64_t( startBit, numBits);
      s <<= 64 - numBits; // Move sign bit to msb.
      s >>= 64- numBits;  // Shift result back to correct location sign bit extended.
      return (s);
   }

   double PackedNavBits::ScaleValue( const double value, const int power2) const
   {
      double temp = value;
      temp /= pow(static_cast<double>(2), power2);
      if (temp >= 0) temp += 0.5; // Takes care of rounding
      else temp -= 0.5;
      return ( temp );
   }
 
   void PackedNavBits::dump(ostream& s) const
      throw()
   {
      ios::fmtflags oldFlags = s.flags();
   
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');
      
      s << "****************************************************************"
        << "************" << endl
        << "Packed Nav Bits" << endl
        << endl
        << "SatID: " << setw(4) << getsatSys() << endl
        << endl
        << "Carrier: " << ObsID::cbDesc[obsID.band] << "      "
        << "Code: " << ObsID::tcDesc[obsID.code] << endl << endl
        << "Number Of Bits: " << dec << getNumBits() << endl
        << endl;
  
      s << "              Week(10bt)     SOW      UTD     SOD"
        << "  MM/DD/YYYY   HH:MM:SS\n";
      s << "  Xmit Time:  ";

      s << printTime( transmitTime, "%4F(%4G) %6.0g      %3j   %5.0s  %02m/%02d/%04Y   %02H:%02M:%02S");
      s << endl;     

      s << endl << "Packed Bits, Left Justified, 32 Bits Long:\n";
      int numBitInWord = 0;
      int word_count   = 0;
      uint32_t word    = 0;
      for(size_t i = 0; i < bits.size(); ++i)
      {
         word <<= 1;
         if (bits[i]) word++;
       
         numBitInWord++;
         if (numBitInWord >= 32)
         {
            s << "  0x" << setw(8) << setfill('0') << hex << word;
            numBitInWord = 0;
            word_count++;
            //Print four words per line 
            if (word_count %5 == 0) s << endl;        
         }
      }
      word <<= 32 - numBitInWord;
      if (numBitInWord > 0 ) s << "  0x" << setw(8) << setfill('0') << hex << word;
      s.setf(ios::fixed, ios::floatfield);
      s.precision(3);
      s.flags(oldFlags);      // Reset whatever conditions pertained on entry

   } // end of PackedNavBits::dump()

      /*
      */
   int PackedNavBits::outputPackedBits(std::ostream& s,
                                       short numPerLine,
                                       char delimiter ) const
   {
      ios::fmtflags oldFlags = s.flags();

      s.setf(ios::uppercase); 
      int rollover = numPerLine;
      
      int numBitInWord = 0;
      int word_count   = 0;
      uint32_t word    = 0;
      for(size_t i = 0; i < bits.size(); ++i)
      {
         word <<= 1;
         if (bits[i]) word++;
       
         numBitInWord++;
         if (numBitInWord >= 32)
         {
            s << delimiter << " 0x" << setw(8) << setfill('0') << hex << word;
            word = 0;
            numBitInWord = 0;
            word_count++;
            
            //Print "numPerLine" words per line 
            if (word_count>0 && 
                word_count % rollover == 0) s << endl;        
         }
      }
         // Need to check if there is a partial word in the buffer
      word <<= 32 - numBitInWord;
      if (numBitInWord>0)
      {
         s << "  0x" << setw(8) << setfill('0') << hex << word;
      }
      s.flags(oldFlags);      // Reset whatever conditions pertained on entry
      return(bits.size()); 
   }

   bool PackedNavBits::matchBits(const PackedNavBits& right, 
                                 short startBit, short endBit) const
   {
         // If the two objects don't have the same number of bits,
         // don't even try to compare them. 
      if (bits.size()!=right.bits.size()) return false; 

         // If not the same satellite, return false. 
      if (satSys!=right.satSys) return false;

         // If not the same observation types (carrier, code) return false.
      if (obsID!=right.obsID) return false;

         // Check for nonsense arguments
      if (endBit==-1 ||
          endBit>=int(bits.size())) endBit = bits.size()-1;
      if (startBit<0) startBit=0;
      if (startBit>=int(bits.size())) startBit = bits.size()-1;

      for (int i=startBit;i<=endBit;i++)
      {
         if (bits[i]!=right.bits[i])
         {
            return false;
         }
      }
      return true;
   }
      
   void PackedNavBits::rawBitInput(const std::string inString )
      throw(InvalidParameter)
   {
         // Debug
      //cout << " ENTERING rawBitInput( )-------------------------------------------------" << endl;
         //  Find first non-white space string.   
         //  Should translate as a decimal value.
         //  If so, assume this is the number of bits that follow, but do not 
         //  store it until success.
         //  For out purposes, treat space, tab, and comma as white space
         //  (the inclusion of comma allows CSV files to be read).
      string whiteSpace=" \t,";
      string::size_type begin = inString.find_first_not_of(whiteSpace);
      if (begin==string::npos)
      {
         InvalidParameter exc("Did not find #bits at beginning of input string.");
         GPSTK_THROW(exc);
      }
      string::size_type end = inString.find_first_of(whiteSpace,begin);
      if (end==string::npos)
      {
         InvalidParameter exc("Did not find space after #bits at beginning of input string.");
         GPSTK_THROW(exc);
      }
      string textBitCount = inString.substr(begin,end);
      int bitsExpected = StringUtils::asInt(textBitCount); 

         //  Find successive 32 bits quantities stored as hex strings.  
         //  That is to say, each should be of the format 0xAAAAAAAA
         //  There should be sufficient to cover the number of input 
         //  bits plus padding to the next even 32 bit word boundary.  
         //  That is to say, []# of 32 bits words] = ((inBits-1)/32) + 1;
      int numWordsExpected = (( bitsExpected-1)/32) + 1; 
      int bitsRead = 0;
         // debug
      //cout << "bitsExpected, numWordsExpected : " << bitsExpected << ", " << numWordsExpected << endl; 
      for (int i = 0; i<numWordsExpected; ++i)
      {
            // For each word, convert the string to a value, then add it
            // to the packed bit storage. 
         begin = inString.find_first_not_of(whiteSpace,end+1);
         if (begin==string::npos)
         {
            InvalidParameter exc("Did not find expected number of hex words.");
            GPSTK_THROW(exc);
         }
         end = inString.find_first_of(whiteSpace,begin);
         string::size_type length = end - begin; 
         string hexWord = inString.substr(begin,length);
            // Debug
         //   cout << "hexWord (string) : '" << hexWord << "'" << endl;
         if (hexWord.substr(0,2)!="0x" &&
             hexWord.substr(0,2)!="0X" )
         {
            InvalidParameter exc("Expected hex data did not being with '0x'");
            GPSTK_THROW(exc); 
         }
            
         unsigned long dataWord = StringUtils::x2uint(hexWord);

            // NOTE: Since the input is always in terms of complete left-
            // justified 32-bit words, the "numberBits" argument to 
            // addUnsignedLong() is always 32.  However, we need to keep
            // track of how many bits are actually being stored in the
            // PackedNavBits object.  The only word that is not 32-bits
            // "full" is the last word. 
         int numBitsToAdd = bitsExpected - bitsRead;
         if (numBitsToAdd>32) numBitsToAdd = 32;
             // Debug
         //    cout << " dataWord (dec) : " << dataWord << endl;
         //    cout << " numBitsToAdd : " << numBitsToAdd << endl;
         addUnsignedLong( dataWord, 32, 1); 

         bitsRead += numBitsToAdd; 

      }
      // Now trim the string and store the final size. 
      bits_used = bitsRead;
      trimsize();  

      return;
   }


   ostream& operator<<(ostream& s, const PackedNavBits& pnb)
   {
      pnb.dump(s);
      return s;

   } // end of operator<<
  
   
} // namespace
