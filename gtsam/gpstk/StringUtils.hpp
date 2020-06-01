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
 * @file StringUtils.hpp
 * StringUtils namespace and GPSTK string utility functions
 */

#ifndef GPSTK_STRINGUTILS_HPP
#define GPSTK_STRINGUTILS_HPP

#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <vector>
#include <cstdio>   /// @todo Get rid of the stdio.h dependency if possible.
#include <cctype>
#include <limits>

#ifdef _WIN32
	#if _MSC_VER < 1700
        // For lower version of visual studio 2012 use gnu regex
		#include <regex.h>
		#pragma comment(lib, "regex.lib")
	#else
        // visual studio 2012 support c++ 0x, and we use std::regex
		#include <regex>
	#endif
#else
    // TODO: we should use std::regex for upper than g++ 4.6
	#include <regex.h>
#endif

#include <gtsam/gpstk/Exception.hpp>

namespace gpstk
{
      /**
       * Stuff to make the C++ string class a little easier to use.  All the
       * functionality here is inlined since they are farily small
       * functions.
       *
       * All functions here will throw gpstk::StringUtils::StringException
       * on an error. Any std::exception is converted to a
       * gpstk::StringUtils::StringException so
       * that's the only exception a user of this class needs to catch.
       *
       * For any function that modifies a string, make sure there is a
       * non-const (std::string&) version and a const (const std::string&)
       * version. The convention for writing the functions is the non-const
       * version fully implements the function and the const version calls
       * the non-const version.
       *
       * @sa stringutiltest.cpp for some examples.
       */
   namespace StringUtils
   {
         /** @defgroup stringutilsgroup Text String Manipulation Tools */
         //@{

         /// This is thrown instread of a std::exception when a
         /// gpstk::StringUtils function fails.
         /// @ingroup exceptiongroup
      NEW_EXCEPTION_CLASS(StringException, Exception);

         /// Class for configuring the appearance of hexDumpData() output
      class HexDumpDataConfig
      {
      public:
         HexDumpDataConfig()
               : showIndex(true), hexIndex(true), upperHex(false),
                 idxDigits(4), indexWS(1), groupBy(1), groupWS(1),
                 group2By(8), group2WS(2), bytesPerLine(16), showText(true),
                 separator(0), textWS(4)
         {}
         HexDumpDataConfig(bool ashowIndex, bool ahexIndex, bool aupperHex,
                           unsigned aidxDigits, unsigned aindexWS,
                           unsigned agroupBy, unsigned agroupWS,
                           unsigned agroup2By, unsigned agroup2WS,
                           unsigned abytesPerLine, bool ashowText,
                           char aseparator, unsigned atextWS)
               : showIndex(ashowIndex), hexIndex(ahexIndex),
                 upperHex(aupperHex), idxDigits(aidxDigits),
                 indexWS(aindexWS), groupBy(agroupBy), groupWS(agroupWS),
                 group2By(agroup2By), group2WS(agroup2WS),
                 bytesPerLine(abytesPerLine), showText(ashowText),
                 separator(aseparator), textWS(atextWS)
         {}
         bool showIndex; ///< display index into string on each line.
         bool hexIndex; ///< if true, use hex index numbers (else decimal).
         bool upperHex; ///< if true, use upper-case hex digits.
         unsigned idxDigits; ///< number of positions to use for index.
         unsigned indexWS; ///< number of whitespace charaters between index and data.
         unsigned groupBy; ///< number of bytes of data to show between spaces.
         unsigned groupWS; ///< number of whitespace charaters between groups of hex data.
         unsigned group2By; ///< number of groups to show per 2nd layer group (0=none, must be multiple of groupBy).
         unsigned group2WS; ///< number of whitespace charaters between 2nd layer groups.
         unsigned bytesPerLine; ///< number of bytes to display on a line of output (must be evenly divisible by both groupBy and group2By).
         bool showText; ///< if true, show text of message (unprintable characters become '.'.
         char separator; ///< character to offset text with (0 = none).
         unsigned textWS; ///< number of whitespace characters between hex and text.
      };

         /**
          * Perform a formatted hex-dump of the (potentially) binary
          * data to the given stream.
          * @param s stream to dump data to.
          * @param data data to hex-dump.
          * @param indent indents the string by that many spaces.
          * @param cfg formatting configuration.
          */
      inline void hexDumpData(std::ostream& s,
                              const std::string& data,
                              unsigned indent = 0,
                              HexDumpDataConfig cfg = HexDumpDataConfig());

         /**
          * Perform a formatted hex-dump of the (potentially) binary
          * data to the given stream.
          * @param s stream to dump data to.
          * @param data data to hex-dump.
          * @param tag string to put at the beginning of each line of output.
          * @param cfg formatting configuration.
          */
      inline void hexDumpData(std::ostream& s,
                              const std::string& data,
                              const std::string& tag,
                              HexDumpDataConfig cfg = HexDumpDataConfig());

         /**
          * Remove a string from the beginning of another string.
          * Occurrences of the string \a aString appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline
      std::string& stripLeading(std::string& s,
                                const std::string& aString,
                                std::string::size_type num = std::string::npos)
         throw(StringException);

         /**
          * Remove a string from the beginning of another string const version.
          * Occurrences of the string \a aString appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripLeading(const std::string& s,
                                      const std::string& aString,
                                      std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripLeading(t, aString, num); return t; }

         /**
          * Remove a string from the beginning of another string.
          * Occurrences of the string \a pString appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripLeading(std::string& s,
                                       const char* pString,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripLeading(s, std::string(pString), num); }

         /**
          * Remove a string from the beginning of another string const version.
          * Occurrences of the string \a pString appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripLeading(const std::string& s,
                                      const char* pString,
                                      std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripLeading(t, std::string(pString), num); return t; }

         /**
          * Strip character(s) from the beginning of a string.
          * Occurrences of the character \a aCharacter appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripLeading(std::string& s,
                                       const char aCharacter,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripLeading(s, std::string(1,aCharacter), num); }

         /**
          * Strip character(s) from the beginning of a string const version.
          * Occurrences of the character \a aCharacter appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripLeading(const std::string& s,
                                      const char aCharacter,
                                      std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripLeading(t, std::string(1,aCharacter), num); return t; }

         /**
          * Strip blanks from the beginning of a string.
          * Occurrences of the space character appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripLeading(std::string& s,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripLeading(s,std::string(1,' '),num); }

         /**
          * Strip blanks from the beginning of a string const version.
          * Occurrences of the space character appearing
          * at the beginning of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripLeading(const std::string& s,
                                      std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripLeading(t,std::string(1,' '),num); return t; }

         /**
          * Remove a string from the end of another string.
          * Occurrences of the string \a aString appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripTrailing(std::string& s,
                                        const std::string& aString,
                                        std::string::size_type num = std::string::npos)
         throw(StringException);

         /**
          * Remove a string from the end of another string const version.
          * Occurrences of the string \a aString appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripTrailing(const std::string& s,
                                       const std::string& aString,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripTrailing(t, aString, num); return t;}

         /**
          * Remove a string from the end of another string.
          * Occurrences of the string \a pString appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripTrailing(std::string& s,
                                        const char* pString,
                                        std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripTrailing(s, std::string(pString), num); }

         /**
          * Remove a string from the end of another string const version.
          * Occurrences of the string \a pString appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripTrailing(const std::string& s,
                                       const char* pString,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripTrailing(t, std::string(pString), num); return t; }

         /**
          * Strip character(s) from the end of a string.
          * Occurrences of the character \a aCharacter appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripTrailing(std::string& s,
                                        const char aCharacter,
                                        std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripTrailing(s, std::string(1,aCharacter), num); }

         /**
          * Strip character(s) from the end of a string const version.
          * Occurrences of the character \a aCharacter appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripTrailing(const std::string& s,
                                       const char aCharacter,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripTrailing(t, std::string(1,aCharacter), num); return t; }

         /**
          * Strip blanks from the end of a string.
          * Occurrences of the space character appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& stripTrailing(std::string& s,
                                        std::string::size_type num = std::string::npos)
         throw(StringException)
      { return stripTrailing(s, std::string(1,' '), num); }

         /**
          * Strip blanks from the end of a string const version.
          * Occurrences of the space character appearing
          * at the end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string stripTrailing(const std::string& s,
                                       std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); stripTrailing(t, std::string(1,' '), num); return t;}

         /**
          * Remove a string from the beginning and end of another string.
          * Occurrences of the string \a aString appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& strip(std::string& s,
                                const std::string& aString,
                                std::string::size_type num = std::string::npos)
         throw(StringException);


         /**
          * Remove a string from the beginning and end of another string const version.
          * Occurrences of the string \a aString appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string strip(const std::string& s,
                               const std::string& aString,
                               std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s);  strip(t, aString, num); return t; }


         /**
          * Remove a string from the beginning and end of another string.
          * Occurrences of the string \a pString appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& strip(std::string& s,
                                const char* pString,
                                std::string::size_type num = std::string::npos)
         throw(StringException)
      { return strip(s, std::string(pString), num); }

         /**
          * Remove a string from the beginning and end of another string cosnt version.
          * Occurrences of the string \a pString appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param pString string to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string strip(const std::string& s,
                               const char* pString,
                               std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s); strip(t, std::string(pString), num); return t; }

         /**
          * Strip character(s) from the beginning and end of a string.
          * Occurrences of the character \a aCharacter appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& strip(std::string& s,
                                const char aCharacter,
                                std::string::size_type num = std::string::npos)
         throw(StringException)
      { return strip(s, std::string(1,aCharacter), num); }

         /**
          * Strip character(s) from the beginning and end of a string const version.
          * Occurrences of the character \a aCharacter appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param aCharacter character to remove.
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string strip(const std::string& s,
                               const char aCharacter,
                               std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s);  strip(t, std::string(1,aCharacter), num); return t;}

         /**
          * Strip blanks from the beginning and end of a string.
          * Occurrences of the space character appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& strip(std::string& s,
                                std::string::size_type num = std::string::npos)
         throw(StringException)
      { return strip(s, std::string(1, ' '), num); }

         /**
          * Strip blanks from the beginning and end of a string const version.
          * Occurrences of the space character appearing
          * at the beginning and end of the string \a s are removed.
          * @param s string to be stripped (modified).
          * @param num maximum number of occurrences to remove.
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string strip(const std::string& s,
                               std::string::size_type num = std::string::npos)
         throw(StringException)
      { std::string t(s);  strip(t, std::string(1, ' '), num); return t;}

         /**
          * Converts all of the receiver's characters that are in the
          * first specified string to the corresponding character in
          * the second specified string.
          * @param aString string to perform translation on.
          * @param inputChars characters in \a aString to translate from.
          * @param outputChars characters to translate to.
          * @param pad pad character in the event inputChars and
          * outputChars are not equal length.  The pad character will
          * become the translated character.
          */
      inline std::string translate(const std::string& aString,
                                   const std::string& inputChars,
                                   const std::string& outputChars,
                                   const char pad = ' ');

         /**
          * Changes occurrences of a specified pattern to a specified
          * replacement string.  You can specify the number of changes
          * to perform.  The default is to change all occurrences of
          * the pattern. You can also specify the position in the
          * receiver at which to begin.
          * @param aString string to perform translation on.
          * @param inputString The pattern string as a reference to an
          *   object of type string.  The library searches for the
          *   pattern string within the receiver's data.
          * @param outputString The replacement string as a reference
          *   to an object of type string. It replaces the occurrences
          *   of the pattern string in the receiver's data.
          * @param startPos The position to start the search at within
          *   the receiver's data.  The default is 0.
          * @param numChanges the number of patterns to search for and
          *   change.  The default is to change all occurrences of the
          *   pattern.
          */
      inline std::string change(const std::string& aString,
                                const std::string& inputString,
                                const std::string& outputString,
                                std::string::size_type startPos = 0,
                                unsigned numChanges = (std::numeric_limits<unsigned>::max)());

         /**
          * Changes occurrences of a specified pattern to a specified
          * replacement string.  You can specify the number of changes
          * to perform.  The default is to change all occurrences of
          * the pattern. You can also specify the position in the
          * receiver at which to begin.
          * @param aString string to perform translation on.
          * @param inputString The pattern string as a reference to an
          *   object of type string.  The library searches for the
          *   pattern string within the receiver's data.
          * @param outputString The replacement string as a reference
          *   to an object of type string. It replaces the occurrences
          *   of the pattern string in the receiver's data.
          * @param startPos The position to start the search at within
          *   the receiver's data.  The default is 0.
          * @param numChanges the number of patterns to search for and
          *   change.  The default is to change all occurrences of the
          *   pattern.
          */
      inline std::string& change(std::string& aString,
                                 const std::string& inputString,
                                 const std::string& outputString,
                                 std::string::size_type startPos = 0,
                                 unsigned numChanges = (std::numeric_limits<unsigned>::max)());

         /**
          * Right-justifies the receiver in a string of the specified
          * length. If the receiver's data is shorter than the
          * requested length (\a length), it is padded on the left with
          * the pad character (\a pad). The default pad
          * character is a blank.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.  */
      inline std::string& rightJustify(std::string& s,
                                       const std::string::size_type length,
                                       const char pad = ' ')
         throw(StringException);

         /**
          * Right-justifies the receiver in a string of the specified
          * length (const version). If the receiver's data is shorter than the
          * requested length (\a length), it is padded on the left with
          * the pad character (\a pad). The default pad
          * character is a blank.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.  */
      inline std::string rightJustify(const std::string& s,
                                      const std::string::size_type length,
                                      const char pad = ' ')
         throw(StringException)
      { std::string t(s); return rightJustify(t, length, pad); }

         /**
          * Left-justifies the receiver in a string of the specified
          * length. If the new length (\a length) is larger than the
          * current length, the string is extended by the pad
          * character (\a pad). The default pad character is a
          * blank.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.  */
      inline std::string& leftJustify(std::string& s,
                                      const std::string::size_type length,
                                      const char pad = ' ')
         throw(StringException);

         /**
          * Left-justifies the receiver in a string of the specified
          * length (const version). If the new length (\a length) is larger
          * than the current length, the string is extended by the pad
          * character (\a pad). The default pad character is a
          * blank.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.  */
      inline std::string leftJustify(const std::string& s,
                                     const std::string::size_type length,
                                     const char pad = ' ')
         throw(StringException)
      { std::string t(s); return leftJustify(t, length, pad); }

         /**
          * Change the length of a string by adding to the beginning and end.
          * The string \a s is modified to the specified
          * length.  If the string is shorter than
          * \a length, then the string is truncated with the
          * left-most \a length characters remaining.
          * Otherwise, characters are added to the beginning and end of the
          * string until the string is the specified length, where the
          * number of characters added to the beginning and the end
          * does not differ by more than one so the original string
          * is centered.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string& center(std::string& s,
                                 const std::string::size_type length,
                                 const char pad = ' ')
         throw(StringException);

         /**
          * Change the length of a string by adding to the beginning and end
          * (const version).
          * The string \a s is modified to the specified
          * length.  If the string is shorter than
          * \a length, then the string is truncated with the
          * left-most \a length characters remaining.
          * Otherwise, characters are added to the beginning and end of the
          * string until the string is the specified length, where the
          * number of characters added to the beginning and the end
          * does not differ by more than one so the original string
          * is centered.
          * @param s string to be modified.
          * @param length new desired length of string.
          * @param pad character to pad string with (blank by default).
          * @throws StringException if there's a std::exception thrown.
          * @return a reference to \a s.
          */
      inline std::string center(const std::string& s,
                                const std::string::size_type length,
                                const char pad = ' ')
         throw(StringException)
      { std::string t(s); return center(t, length, pad); }

         /**
          * Convert a string to a double precision floating point number.
          * @param s string containing a number.
          * @return double representation of string.
          */
      inline double asDouble(const std::string& s)
      { return strtod(s.c_str(), 0); }

         /**
          * Convert a string to an integer.
          * @param s string containing a number.
          * @return long integer representation of string.
          */
      inline long asInt(const std::string& s)
      { return strtol(s.c_str(), 0, 10); }

         /**
          * Convert a string to an unsigned integer.
          * @param s string containing a number.
          * @return unsigned long integer representation of string.
          */
      inline unsigned long asUnsigned(const std::string& s)
      { return strtoul(s.c_str(), 0, 10); }

         /**
          * Convert a string to a single precision floating point number.
          * @param s string containing a number.
          * @return single representation of string.
          */
      inline float asFloat(const std::string& s)
         throw(StringException);

         /**
          * Convert a string to a big precision floating point number.
          * @param s string containing a number.
          * @return long double representation of string.
          */
      inline long double asLongDouble(const std::string& s)
         throw(StringException);

         /**
          * Convert a value in a string to a type specified by the template
          * class.  The template class type must have stream operators
          * defined.
          * @param s object to turn into the templatized type.
          * @return the template object of \a x.
          */
      template <class X>
      inline X asData(const std::string& s)
         throw(StringException);

         /**
          * Convert a long double to a string in fixed notation.
          * @param x long double.
          * @param precision the number of decimal places you want displayed.
          * @return string representation of \a x.
          */
      inline std::string asString(const long double x,
                             const std::string::size_type precision = 21);

         /**
          * Convert a double to a string in fixed notation.
          * @param x double.
          * @param precision the number of decimal places you want displayed.
          * @return string representation of \a x.
          */
      inline std::string asString(const double x,
                             const std::string::size_type precision = 17);

         /**
          * Convert any old object to a string.
          * The class must have stream operators defined.
          * @param x object to turn into a string.
          * @return string representation of \a x.
          */
      template <class X>
      inline std::string asString(const X x);

         /**
          * Convert a decimal string to a hexadecimal string.
          * Modify the string such that the decimal integer is now
          * represented as hexadecimal.  Only the first decimal encountered is
          * changed; the rest of the string is unmodified.
          * @param s string containing an integer.
          * @return reference to modified \a s.
          */
      inline std::string& d2x(std::string& s)
         throw(StringException);

         /**
          * Convert a decimal string to a hexadecimal string.
          * Given a string containing a decimal integer, convert the
          * integer from base 10 to base 16 and return the result.  No
          * prefix is added.  Only the first decimal encountered is
          * changed; the rest of the string is unmodified.
          * @param s string containing an integer.
          * @return string containing a hexadecimal number.
          */
      inline std::string d2x(const std::string& s)
         throw(StringException)
      { std::string t(s);  return d2x(t); }

         /**
          * Convert a hexadecimal string to a decimal string.
          * Modify the string such that the hexadecimal number is now
          * represented as decimal.  Only the first hex number encountered
          * is changed; the rest of the string is unmodified.
          * @param s string containing an integer.
          * @return reference to modified \a s.
          */
      inline std::string& x2d(std::string& s)
         throw(StringException);

         /**
          * Convert a hexadecimal string to a decimal string.
          * Given a string containing a hexadecimal number, convert the
          * integer from base 16 to base 10 and return the result.
          * Only the first hex number encountered
          * is changed; the rest of the string is unmodified.
          * @param s string containing an integer.
          * @return string containing a hexadecimal number.
          */
      inline std::string x2d(const std::string& s)
         throw(StringException)
      { std::string t(s);  return x2d(t); }

         /**
          * Convert a character string to a hexadecimal string.
          * Modify the string such that the character string is now
          * represented as series of hexadecimal digits.
          * @param s string to convert.
          * @return reference to modified \a s.
          */
      inline std::string& c2x(std::string& s)
         throw(StringException);

         /**
          * Convert a character string to a hexadecimal string.
          * @param s string containing an integer.
          * @return string containing a sequence of hexadecimal numbers.
          */
      inline std::string c2x(const std::string& s)
         throw(StringException)
      { std::string t(s);  return c2x(t); }

         /**
          * Convert a hexadecimal string to an unsigned int.
          * Only the first hex number encountered is converted.
          * @param s string containing a hex integer.
          * @return a long holding the value of \a s.
          */
      inline unsigned int x2uint(const std::string& s)
         throw(StringException);

         /**
          * Convert an int to a string.
          * @param i the integer to convert
          * @return a string with the hex equivalent of i
          */
      inline std::string int2x(const unsigned int& i)
         throw(StringException);

         /**
          * Replace all instances of \a oldString with \a newString in \a s.
          * @param s the string whose contents will be modified.
          * @param oldString the string to search for in \a s.
          * @param newString the string to replace \a oldString in \a s.
          * @return a reference to the modified string.
          */
      inline std::string& replaceAll(std::string& s,
                                     const std::string& oldString,
                                     const std::string& newString )
         throw(StringException);

         /**
          * isDigitString is exactly like the C function isDigit
          * except it checks all the characters of string \a s to see if
          * they are all digits.
          * @param s the string to check the digits in.
          * @return true if \a s is all digits, false otherwise.
          */
      inline bool isDigitString(const std::string& s);

         /**
          * isDecimalString is like isDigitString() except it allows a
          * single period ('.') character in the string.
          * @param s the string to check.
          * @return true if \a s is a valid fixed-point number.
          */
      inline bool isDecimalString(const std::string& s);

         /**
          * isScientificString extends isDecimalString() to allow a single
          * exponent (E,e,D,d) character between a decimal string and
          * a (possibly empty) digit string.
          * @param s the string to check.
          * @return true if \a s is a valid scientific-notation number.
          */
      inline bool isScientificString(const std::string& s);

         /**
          * isAlphaString is exactly like the C function isAlpha
          * except it checks all the characters of string \a s to see if
          * they are all alphabet characters.
          * @param s the string to check the characters in.
          * @return true if \a s is all digits, false otherwise.
          */
      inline bool isAlphaString(const std::string& s);

         /**
          * Perform pattern matching on strings.
          * Looks for a pattern in a string.  Wildcards are allowed.
          * Uses POSIX regular expressions.
          * @param s string to search.
          * @param aPattern pattern to search for. This is a POSIX
          * regular expression.
          * @param zeroOrMore character representing wildcards
          * matching strings of zero or more characters (default '*').
          * @param oneOrMore character representing plus sign
          * matching strings of one or more characters (default '+').
          * @param anyChar character representing wildcards matching a
          * single arbitrary character (default '.').
          * @return string representing the first match of \a aPattern in
          * \a s.  Returns a null string if no match is found.
          */
      inline std::string matches(const std::string& s,
                                 const std::string& aPattern,
                                 const char zeroOrMore = '*',
                                 const char oneOrMore = '+',
                                 const char anyChar = '.' )
         throw(StringException);

         /**
          * Perform pattern matching on strings.
          * Looks for a pattern in a string.  Wildcards are allowed.
          * Uses POSIX regular expressions.
          * @param s string to search.
          * @param aPattern pattern to search for. This is a POSIX
          * regular expression.
          * @param zeroOrMore character representing wildcards
          * matching strings of zero or more characters (default '*').
          * @param oneOrMore character representing plus sign
          * matching strings of one or more characters (default '+').
          * @param anyChar character representing wildcards matching a
          * single arbitrary character (default '.').
          * @return t if a match is found, f if not.
          */
      inline bool isLike(const std::string& s,
                         const std::string& aPattern,
                         const char zeroOrMore = '*',
                         const char oneOrMore = '+',
                         const char anyChar = '.' )
         throw(StringException)
      { return matches(s, aPattern, zeroOrMore, oneOrMore, anyChar) !=
           std::string(); }


         /**
          * Perform pattern matching on strings.
          * Looks for a pattern in a string.  Wildcards are allowed.
          * Uses POSIX regular expressions.
          * @param s string to search.
          * @param pPattern pattern to search for. This is a POSIX
          * regular expression.
          * @param zeroOrMore character representing wildcards
          * matching strings of zero or more characters (default '*').
          * @param oneOrMore character representing plus sign
          * matching strings of one or more characters (default '+').
          * @param anyChar character representing wildcards matching a
          * single arbitrary character (default '.').
          * @return t if a match is found, f if not.
          */
      inline bool isLike(const std::string& s,
                         const char* pPattern,
                         const char zeroOrMore = '*',
                         const char oneOrMore = '+',
                         const char anyChar = '.' )
         throw(StringException)
      { return matches(s, std::string(pPattern),
                       zeroOrMore, oneOrMore, anyChar) !=  std::string(); }


         /**
          * Work-horse method for printf.  Substitutes patterns
          * matching \a pat with \a rep.  Use only one pattern/token
          * at a time!  This used to be DayTime::iprint().
          * @param fmt format to use for this time.
          * @param pat regular expression pattern to match.
          * @param rep sprintf token replacement.  First character is
          * token character used in fmt, remainder is sprintf token to
          * use.  For example, with fmt="%15S", pat="%[ 0-]?[[:digit:]]*S",
          * and rep="Sd", the fmt will be translated to "%15d" before
          * using it in a sprintf call like printf("%15d"), \a to.
          * @param to the value to stuff into the string.
          * @return \a fmt with \a pat replaced by \a to.  If there is no
          * match, \a fmt is returned unchanged.
          */
      template <class T>
      std::string formattedPrint(const std::string& fmt,
                                 const std::string& pat,
                                 const std::string& rep,
                                 T to)
         throw(StringException);

         /**
          * Get a substring of a string.
          * Try to avoid using this, use the stl string's substr
          * method instead (and ::leftJustify if needed).
          */
      inline std::string subString(const std::string& s,
                                   const std::string::size_type startPos = 0,
                                   const std::string::size_type length = std::string::npos,
                                   const char pad = ' ' )
         throw(StringException);

         /**
          * Change all upper-case letters in a string to lower-case.
          * \a s is modified as a result.
          * @param s string to change to lower case.
          * @return a copy of the original string, all in lower-case.
          */
      inline std::string& lowerCase(std::string& s);

         /**
          * Change all upper-case letters in a string to lower-case.
          * Does not modify the original string.
          * @param s a string containing upper-case characters.
          * @return a copy of the original string, all in lower-case.
          */
      inline std::string lowerCase(const std::string& s)
      { std::string t(s);  return lowerCase(t); }

         /**
          * Change all lower-case letters in a string to upper-case.
          * \a s is modified as a result.
          * @param s string to change to upper case.
          * @return a copy of the original string, all in upper-case.
          */
      inline std::string& upperCase(std::string& s);

         /**
          * Change all lower-case letters in a string to upper-case.
          * Does not modify the original string.
          * @param s a string containing lower-case characters.
          * @return a copy of the original string, all in upper-case.
          */
      inline std::string upperCase(const std::string& s)
      { std::string t(s);  return upperCase(t); }

         /**
          * Make a string from a void pointer.
          * This function should not be used.  Instead, use the string
          * constructor as follows:
          * \code string((char*)p, size); \endcode
          * @param p pointer to memory.
          * @param size length of the data to turn into a string.
          * @return string object containing the contents of \a p.
          */
      inline std::string memToString(const void* p,
                                     const std::string::size_type size);

         /**
          * Returns the first word in string \a s without modifying the string.
          * @param s the string to count the words from.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return the first word from \a s;
          */
      inline std::string firstWord(const std::string& s,
                                   const char delimiter = ' ')
         throw(StringException);

         /**
          * Counts the number of words in \a s and returns it.
          * @param s the string to count the words from.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return the number of words in \a s.
          */
      inline int numWords(const std::string& s,
                          const char delimiter = ' ')
         throw(StringException);

         /**
          * Returns \a numWords words starting with \a firstWord from
          * \a s (if any).
          * @param s a string with the word you want removed.
          * @param firstWord the number of the first word you want from \a s.
          * The first word is word 0.
          * @param numWords number of words to get from \a s.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return the first word from \a s or an empty string if there is
          * no \a wordNum'th word.
          */
      inline std::string words(const std::string& s,
                               const std::string::size_type firstWord = 0,
                               const std::string::size_type numWords = std::string::npos,
                               const char delimiter = ' ')
         throw(StringException);

         /**
          * Returns word number \a wordNum from \a s (if any).
          * @param s a string with the word you want removed.
          * @param wordNum the number of the word you want from \a s.
          * The first word is word 0.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return the first word from \a s or an empty string if there is
          * no \a wordNum'th word.
          */
      inline std::string word(const std::string& s,
                              const std::string::size_type wordNum = 0,
                              const char delimiter = ' ')
         throw(StringException)
      { return words(s, wordNum, 1, delimiter); }

         /**
          * Removes the first word off string \a s and returns it.
          * \a s is modified as a result.
          * @param s a string with the word you want removed.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return the first word from \a s
          */
      inline std::string stripFirstWord(std::string& s,
                                        const char delimiter = ' ')
         throw(StringException);

         /**
          * Split a string \a str into words as defined by \a delimiter.
          * @param str string to be parsed.
          * @param delimiter character that marks the start and end of a word.
          * @return a vector of the words (strings)
          */
       inline std::vector<std::string> split(const std::string& str,
                                             const char delimiter = ' ')
          throw(StringException);

         /**
          * Removes indicated words from the string \a s.
          * \a s is modified as a result.
          * @param s a string with the words you want removed.
          * @param first the first word to be removed (the first word is 0).
          * @param wordsToReplace the number of words you want removed.
          * @param delimiter the character that marks the start and
          * end of a word.
          * @return a reference to string \a s with the words removed.
          */
      inline std::string& removeWords(std::string& s,
                                      const std::string::size_type first = 0,
                                      const std::string::size_type wordsToReplace = std::string::npos,
                                      const char delimiter = ' ')
         throw(StringException);

         /**
          * Convert a double to a scientific notation number.
          * @param d the double to convert
          * @param length length (in characters) of output, including exponent
          * @param expLen length (in characters) of the exponent, with sign
          * @param showSign if true, reserves 1 character for +/- sign
          * @param checkSwitch if true, keeps the exponential sanity check for
          * exponentials above three characters in length.  If false, it removes
          * that check.
          */
      inline std::string doub2sci(const double& d,
                                  const std::string::size_type length,
                                  const std::string::size_type expLen,
                                  const bool showSign = true,
                                  const bool checkSwitch = true);

         /** Convert a double to scientific notation; this routine works better,
          * on Windows particularly, than doub2sci.
          * @param length = total string length,
          *                         including 1 for overall sign if showPlus is true.
          * @param precision = number of digits after the decimal and before the 'e'
          * @param explen = length of exponent, this must = 1, 2 or 3
          * NB. length is increased if precision, explen and showPlus require it.
          */
      inline std::string doubleToScientific(const double& d,
                                            const std::string::size_type length,
                                            const std::string::size_type precision,
                                            const std::string::size_type explen,
                                            bool showPlus=false);

         /**
          * Convert scientific notation to FORTRAN notation.
          * As an example, the string "1.5636E5" becomes " .15636D6".
          * Note that the first character of the string will be '-' if
          * the number is negative or ' ' if the first character is positive.
          * @param aStr string with number to convert
          * @param startPos start position of number in string
          * @param length length (in characters) of number, including exponent.
          * @param expLen length (in characters of exponent, not including sign.
          * @param checkSwitch will keep the method running as orignially programed
          * when set to true.  If false, the method will always resize exponentials,
          * produce an exponential with an E instead of a D, and always have a leading
          * zero.  For example -> 0.87654E-0004 or -0.1234E00005.
          * @throws Exception if the string is not a number in scientific notation
          */
      inline std::string& sci2for(std::string& aStr,
                                  const std::string::size_type startPos = 0,
                                  const std::string::size_type length = std::string::npos,
                                  const std::string::size_type expLen = 3,
                                  const bool checkSwitch = true)
         throw(StringException);

         /**
          * Convert double precision floating point to a string
          * containing the number in FORTRAN notation.
          * As an example, the number 156360 becomes ".15636D6".
          * @param d number to convert.
          * @param length length (in characters) of number, including exponent.
          * @param expLen length (in characters of exponent, including sign.
          * @param checkSwitch if true, keeps the exponential sanity check for
          * exponentials above three characters in length.  If false, it removes
          * that check.
          * @return a string containing \a d in FORTRAN notation.
          */
      inline std::string doub2for(const double& d,
                                  const std::string::size_type length,
                                  const std::string::size_type expLen,
                                  const bool checkSwitch = true)
         throw(StringException);

         /**
          * Convert FORTRAN representation of a double precision
          * floating point in a string to a number.
          * As an example, the number ".15636D6" becomes 156360.
          * @param aStr string containing FORTRAN representation of number.
          * @param startPos beginning of number in string.
          * @param length length (in characters) of number, including exponent.
          * @return value of the number.
          */
      inline double for2doub(const std::string& aStr,
                             const std::string::size_type startPos = 0,
                             const std::string::size_type length = std::string::npos);

         /**
          * Change a string into printable characters.  Control
          * characters (0-26) are changed to ^@, ^A, etc.  Other
          * non-printable characters are changed to hex sequences
          * enclosed in <>.
          * @param aStr the string to make printable.
          */
      inline std::string printable(const std::string& aStr)
         throw(StringException);

         /**
          * Nicely expands the input string into several lines, non-const
          * version.
          * @param aStr the string to be modified.
          * @param lineDelim a string to put between every line.
          * @param indent an indentataion string used on all but the first line
          * @param firstIndent is the indentation used on the first line.
          * @param len the maximum length of string to put on a line.
          * @param wordDelim the character that separates each word.
          * @return the string nicely formatted.
          */
      inline std::string& prettyPrint(std::string& aStr,
                                      const std::string& lineDelim = "\n",
                                      const std::string& indent = "",
                                      const std::string& firstIndent = "     ",
                                      const std::string::size_type len = 80,
                                      const char wordDelim = ' ')
         throw(StringException);

         /**
          * Const version of prettyPrint, which nicely expands the
          * input string into several lines.
          * @param aStr the string to be modified.
          * @param lineDelim a string to put between every line.
          * @param indent an indentataion string used on all but the first line
          * @param firstIndent is the indentation used on the first line.
          * @param len the maximum length of string to put on a line.
          * @param wordDelim the character that separates each word.
          * @return the string nicely formatted.
          */
      inline std::string prettyPrint(const std::string& aStr,
                                     const std::string& lineDelim = "\n",
                                     const std::string& indent = "",
                                     const std::string& firstIndent = "     ",
                                     const std::string::size_type len = 80,
                                     const char wordDelim = ' ')
         throw(StringException)
      {
         std::string temp(aStr);
         prettyPrint(temp, lineDelim, indent, firstIndent, len, wordDelim);
         return temp;
      }

         /** Split a string by some delimiters
          * @param  aStr           the string to be splitted
          * @param  theDelimiters  the delimiters to split the string
          * @param  trimWhitespace will trim the token string, default is false
          * @param  ignoreEmpty    will ignore the empty tokens, default is true
          */
      inline std::vector<std::string> split(const std::string& aStr,
                                            const std::string& theDelimiters=" ",
                                            bool trimWhitespace = false,
                                            bool ignoreEmpty = true)
      {
         std::vector<std::string> toReturn;

         std::string::size_type lastPos = aStr.find_first_not_of(theDelimiters, 0);
         std::string::size_type pos     = aStr.find_first_of(theDelimiters, lastPos);

         while (std::string::npos != pos || std::string::npos != lastPos)
         {
            std::string token = aStr.substr(lastPos, pos - lastPos);

            if(trimWhitespace) token = StringUtils::strip(token);

            if(!token.empty() || !ignoreEmpty) toReturn.push_back(token);

            lastPos = aStr.find_first_not_of(theDelimiters, pos);
            pos = aStr.find_first_of(theDelimiters, lastPos);
         }

         return toReturn;
      }

   } // namespace StringUtils

} // namespace gpstk

// ################################################
//   Implementations of inline functions follow
// ################################################

namespace gpstk
{

   namespace StringUtils
   {
      inline void hexDumpData(std::ostream& s, const std::string& data,
                              unsigned indent, HexDumpDataConfig cfg)
      {
         std::string instr(indent, ' ');
         hexDumpData(s, data, instr, cfg);
      }

      inline void hexDumpData(std::ostream& s, const std::string& data,
                              const std::string& tag, HexDumpDataConfig cfg)
      {
         std::string ascii="";
         unsigned indent = tag.length();
         int col = 0;
         int datasize=data.size();
         std::string groupws(cfg.groupWS, ' ');
         std::string group2ws(cfg.group2WS, ' ');
         std::string indexws(cfg.indexWS, ' ');
         std::string textws(cfg.textWS, ' ');
         unsigned linesize;

         if (cfg.groupBy && ((cfg.bytesPerLine % cfg.groupBy) != 0))
         {
            s << "hexDumpData: cfg.bytesPerLine % cfg.groupBy != 0"
              << std::endl;
            return;
         }
         if (cfg.group2By && ((cfg.bytesPerLine % cfg.group2By) != 0))
         {
            s << "hexDumpData: cfg.bytesPerLine % cfg.group2By != 0"
              << std::endl;
            return;
         }
         if (cfg.groupBy && ((cfg.group2By % cfg.groupBy) != 0))
         {
            s << "hexDumpData: cfg.group2By % cfg.groupBy != 0"
              << std::endl;
            return;
         }

            // line format:
            // <tag><index>:<indexws><group1byte1>...<group1byte[groupBy]><groupws>...<group[group2By]byte1>...<group[group2By]byte[groupBy]><group2ws>....<byte[bytesPerLine]><textws><separator><text><separator>\n
         linesize = indent;
         if (cfg.showIndex)
            linesize += cfg.idxDigits + 1 + cfg.indexWS;
         linesize += cfg.bytesPerLine * 2;
         unsigned w2 = 0;
         unsigned w1 = 0;
         if (cfg.group2By)
            w2 = (cfg.bytesPerLine / cfg.group2By) - 1;
         if (cfg.groupBy)
            w1 = (cfg.bytesPerLine / cfg.groupBy) - w2 - 1;
         if (cfg.groupBy > 0)
            linesize += cfg.groupWS * w1;
         if (cfg.group2By > 0)
            linesize += cfg.group2WS * w2;
            /*
              linesize doesn't include text stuff
         if (cfg.showText)
            linesize += cfg.textWS + cfg.bytesPerLine;
         if (cfg.separator)
            linesize += 2;
            */

         for (int i=0; i<datasize; i++)
         {
            if (i%cfg.bytesPerLine==0)
            {
               s << tag;
               col = indent;
               if (cfg.showIndex)
               {
                  if (cfg.hexIndex)
                  {
                     s << std::hex;
                     if (cfg.upperHex)
                        s << std::uppercase;
                     else
                        s << std::nouppercase;
                  }
                  else
                     s << std::dec;
                  s << std::setfill('0');
                  s << std::setw(cfg.idxDigits) << i << ":" << indexws;
                  s << std::dec << std::nouppercase;
               }
               col += cfg.idxDigits + 1 + cfg.indexWS;
            }
            unsigned char c=data[i];
            if (isprint(c))
               ascii += c;
            else
               ascii += '.';
            if (cfg.upperHex)
               s << std::uppercase;
            else
               s << std::nouppercase;
            s << std::hex << std::setw(2) << (int)c << std::dec
              << std::nouppercase;
            col += 2;
            if (((i % cfg.bytesPerLine) == (cfg.bytesPerLine-1)) ||
                (i == (datasize-1)))
            {
               if (cfg.showText)
               {
                  int extra = linesize-col;
                  std::string space(extra, ' ');
                  s << space << textws;
                  if (cfg.separator)
                     s << cfg.separator;
                  s << ascii;
                  if (cfg.separator)
                     s << cfg.separator;
                  s << std::endl;
               }
                  // this *should* be updated at the beginning of the loop
                  //col=indent+6;
               ascii.erase();
            }
            else if (cfg.group2By && ((i % cfg.group2By) == (cfg.group2By-1)))
            {
               s << group2ws;
               col += cfg.group2WS;
            }
            else if (cfg.groupBy && ((i % cfg.groupBy) == (cfg.groupBy-1)))
            {
               s << groupws;
               col += cfg.groupWS;
            }
         }
      }

         // Keep searching for aString at the start of s
         // until num == 0 or aString is not found at the start of s
      inline std::string& stripLeading(std::string& s,
                                  const std::string& aString,
                                  std::string::size_type num)
         throw(StringException)
      {
         try
         {
            if (aString == "")
               return s;

            while((num > 0) &&
                  (s.find(aString,0) == 0) &&
                  (s.length() > 0))
            {
               s.erase(0,aString.length());
               --num;
            }
            return s;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         // keep searching for aString at the end of s
         // until aString isn't found there or num == 0
      inline std::string& stripTrailing(std::string& s,
                                   const std::string& aString,
                                   std::string::size_type num)
         throw(StringException)
      {
         try
         {
            std::string::size_type pos = s.length() - aString.length();

               // empty string, etc.
            if ((pos > s.length()) || (aString == ""))
               return s;

            while((num > 0) &&
                  (s.rfind(aString,pos) == pos) &&
                  (s.length() > 0))
            {
               s.erase(pos, std::string::npos);
               --num;
               pos = s.length() - aString.length();
            }
            return s;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string& strip(std::string& s,
                           const std::string& aString,
                           std::string::size_type num)
         throw(StringException)
      {
         stripLeading(s, aString, num);
         stripTrailing(s, aString, num);
         return s;
      }

      inline std::string translate(const std::string& aString,
                              const std::string& inputChars,
                              const std::string& outputChars,
                              const char pad)
      {
         std::string rv = aString;
         std::string::size_type aspos = 0;
         std::string::size_type inpos = std::string::npos;
         char toc = pad;

            // By starting at the last position, we avoid infinite
            // loops in case someone did something dumb, like, for
            // example, setting inputChars=outputChars.
         while ((aspos = rv.find_first_of(inputChars, aspos))
                != std::string::npos)
         {
               // figure out which char we found;
            inpos = inputChars.find(rv[aspos]);
            if (outputChars.length()-1 < inpos)
               toc = pad;
            else
               toc = outputChars[inpos];
            rv[aspos] = toc;

            aspos++; // try to guarantee no infinite loops
         }

         return rv;
      }

      inline std::string change(const std::string& aString, const std::string& inputString,
                           const std::string& outputString,
                           std::string::size_type startPos, unsigned numChanges)
      {
         std::string rv(aString);
         change(rv, inputString, outputString, startPos, numChanges);
         return rv;
      }

      inline std::string& change(std::string& aString, const std::string& inputString,
                            const std::string& outputString,
                            std::string::size_type startPos, unsigned numChanges)
      {
    unsigned count = 0;
         std::string::size_type opos = startPos;

         while (count < numChanges)
         {
            std::string::size_type pos = aString.find(inputString, opos);
            if (pos != std::string::npos)
            {
               count++;
               aString.replace(pos, inputString.length(), outputString);
               opos = pos + outputString.length();
            }
            else
               break;
         }

         return aString;
      }

         // if the string is bigger than length, truncate it from the left.
         // otherwise, add pad characters to it's left.
      inline std::string& rightJustify(std::string& s,
                                  const std::string::size_type length,
                                  const char pad)
         throw(StringException)
      {
         try
         {
            if(length < s.length())
            {
               s = s.substr(s.length()-length, std::string::npos);
            }
            else
            {
               s.insert((std::string::size_type)0, length-s.length(), pad);
            }
            return s;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         // if the string is bigger than length, truncate it from the right.
         // otherwise, add pad characters to it's right.
      inline std::string& leftJustify(std::string& s,
                                 const std::string::size_type length,
                                 const char pad)
         throw(StringException)
      {
         try
         {
            if(length < s.length())
            {
               s = s.substr(0, length);
            }
            else
            {
               s.append(length-s.length(), pad);
            }
            return s;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         // leftJustify if s is bigger than length.
         // otherwise, add pad to the left and right of s.
      inline std::string& center(std::string& s,
             const std::string::size_type length,
             const char pad)
         throw(StringException)
      {
         try
         {
            if(length < s.length())
            {
               leftJustify(s, length, pad);
            }
            else {
               std::string::size_type leftOff = s.length() + (length - s.length()) / 2;
               leftJustify(s, leftOff, pad);
               rightJustify(s, length, pad);
            }
            return s;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }


      inline float asFloat(const std::string& s)
         throw(StringException)
      {
         try
         {
            std::istringstream is(s);
            float f;
            is >> f;
            return f;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline long double asLongDouble(const std::string& s)
         throw(StringException)
      {
         try
         {
            std::istringstream is(s);
            long double x;
            is >> x;
            return x;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      template <class X>
      inline X asData(const std::string& s)
         throw(StringException)
      {
         try
         {
            std::istringstream is(s);
            X x;
            is >> x;
            return x;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string asString(const long double x, const std::string::size_type precision)
      {
         std::ostringstream ss;
         ss << std::fixed << std::setprecision(precision) << x ;
         return ss.str();
      }

      inline std::string asString(const double x, const std::string::size_type precision)
      {
         std::ostringstream ss;
         ss << std::fixed << std::setprecision(precision) << x;
         return ss.str();
      }

      template<class X>
      inline std::string asString(const X x)
      {
         std::ostringstream ss;
         ss << x;
         return ss.str();
      }

         // decimal to hex...
      inline std::string& d2x(std::string& s)
         throw(StringException)
      {
         try
         {
               // remove the integer from s, including
               // leading spaces and 0's
            long l = asInt(s);
            stripLeading(s);
            stripLeading(s, "0");
            stripLeading(s, asString<long>(l));

               // put the int in a stringstream to convert it
            std::ostringstream st;
            st << std::hex << l << std::dec;

               // add the new hex to s
            s.insert(0, upperCase(st.str()) );

            return s;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         // character to hex...
      inline std::string& c2x(std::string& s)
         throw(StringException)
      {
         const char hexDigits[] = "0123456789ABCDEF";
         try
         {
            std::string old(s);
            const unsigned char *pSource = (unsigned char *)old.c_str();
            unsigned n = old.length();

            s.resize(n * 2, 0);

            for (int i = 0; i < (int)n * 2;)
            {
               unsigned char c = *pSource++;
               s[i++] = hexDigits[ c / 16 ];
               s[i++] = hexDigits[ c % 16 ];
            }

            return s;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         /// @todo Need to find a way to combine this with x2d.
          // hex to a long.
      inline unsigned int x2uint(const std::string& s)
         throw (StringException)
      {
         try
         {
               // make the stringstream, get the integer, and
               // remove it from the string
            std::istringstream iss(s);
            unsigned int ui;
            iss >> std::hex >> ui;
            return ui;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         /// @todo detecting 0 isn't quite right...
         // hex to decimal
      inline std::string& x2d(std::string& s)
         throw(StringException)
      {
         try
         {
               // remove the "0x" part, leading zeros and spaces from the
               // string
               // ex. ' 0x003' -> '3'
            stripLeading(s);
            stripLeading(s, "0x", 1);
            stripLeading(s, "0");

               // make the stringstream, get the integer, and
               // remove it from the string
            std::istringstream strstr(s);
            int i = 0;
            strstr >> std::hex >> i;
            stripLeading(s, asString<int>(asInt(s)), 1);

               // append the decimal to the existing string
            s.insert(0,asString<int>(i));
            return s;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string int2x(const unsigned int& i)
         throw(StringException)
      {
         try
         {
            std::ostringstream ss;
            ss << std::hex << i;
            return ss.str();
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string& replaceAll(std::string& s,
                                const std::string& oldString,
                                const std::string& newString)
         throw(StringException)
      {
         try
         {
            int spot = s.find(oldString, 0);
            while (spot != (int)std::string::npos)
            {
               s.replace(spot, oldString.length(), newString);
               spot += newString.length();
               spot = s.find(oldString, spot);
            }
            return s;
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline bool isDigitString(const std::string& s)
      {
         if (s.size() == 0)
            return false;

         std::string::size_type index = 0;
         if((s[0] == '-') || (s[0] == '+'))
            index++;
         for( ; index < s.size(); index++)
            if (!isdigit(s[index]))
               return false;
         return true;
      }

      inline bool isDecimalString(const std::string& s)
      {
         if (s.size() == 0)
            return false;

         std::string::size_type index = 0;
         bool sawdot = false;
         if((s[0] == '-') || (s[0] == '+'))
            index++;
         for( ; index < s.size(); index++)
         {
            if (s[index] == '.')
            {
               if (sawdot)
                  return false;
               else sawdot = true;
            }
            else if (!isdigit(s[index]))
               return false;
         }
         return true;
      }

      inline bool isScientificString(const std::string& s)
      {
         if(s.size() == 0)
            return false;

         std::string::size_type pos = s.find_first_of("EeDd");
         if(pos == std::string::npos)
            return isDecimalString(s);

         std::string mant=s.substr(0,pos);
         std::string exp=s.substr(pos+1);
         return (isDecimalString(mant) && (exp.size()==0 || isDigitString(exp)));
      }

      inline bool isAlphaString(const std::string& s)
      {
         if (s.size() == 0)
            return false;

         std::string::size_type index;
         for(index = 0; index < s.size(); index++)
            if (!isalpha(s[index]))
               return false;
         return true;
      }

      inline std::string matches(const std::string& s,
                                 const std::string& aPattern,
                                 const char zeroOrMore,
                                 const char oneOrMore,
                                 const char anyChar)
         throw(StringException)
      {
		  std::string thisPattern(aPattern);
		  std::string thisStr(s);

		  // check if something other than the regex standard
		  // characters (*,+,.) is used for those variables
		  if (zeroOrMore != '*')
		  {
			  replaceAll(thisPattern, "*", "\\*");
			  replaceAll(thisPattern, std::string(1, zeroOrMore), "*");
		  }
		  if (oneOrMore != '+')
		  {
			  replaceAll(thisPattern, "+", "\\+");
			  replaceAll(thisPattern, std::string(1, oneOrMore), "+");
		  }
		  if (anyChar != '.')
		  {
			  replaceAll(thisPattern, ".", "\\.");
			  replaceAll(thisPattern, std::string(1, anyChar), ".");
		  }

#if defined(_WIN32) && _MSC_VER >= 1700
		  try
		  {
			  std::regex reg (thisPattern);

			  std::smatch sm;
			  if(std::regex_search(thisStr,sm,reg,
				  std::regex_constants::match_not_bol|
				  std::regex_constants::match_not_eol))
			  {
				  return sm.str();
			  }
			  else
			  {
				  return std::string();
			  }
		  }
		  catch(std::regex_error& e)
		  {
			  Exception E(std::string("std::regex_error: ") + e.what() );
			  GPSTK_THROW(E);
		  }

#else

         const std::string::size_type regErrorBufSize = 512;


         regmatch_t matches;
         regex_t regExp;
         char errorMsg[regErrorBufSize];
         int rc = regcomp(&regExp, thisPattern.c_str(), REG_EXTENDED);

         if (rc != 0)
         {
            regerror(rc, NULL, errorMsg, regErrorBufSize - 1);
            regfree(&regExp);
            StringException strerr("Regexp error: " + std::string(errorMsg));
            GPSTK_THROW(strerr);
         }
         rc = regexec(&regExp, thisStr.c_str(), 1, &matches,
                      REG_NOTBOL | REG_NOTEOL);
         if ( (rc != 0) && (rc != REG_NOMATCH) )
         {
            regerror(rc, &regExp, errorMsg, regErrorBufSize - 1);
            regfree(&regExp);
            StringException strerr("Regexp error: " + std::string(errorMsg));
            GPSTK_THROW(strerr);
         }

         regfree(&regExp);
         if (rc == REG_NOMATCH)
            return std::string();
         else
            return thisStr.substr(matches.rm_so, matches.rm_eo - matches.rm_so);
#endif
      }

      template <class T>
      inline std::string formattedPrint(const std::string& fmt,
                                        const std::string& pat,
                                        const std::string& rep,
                                        T to)
         throw(StringException)
      {
#if defined(_WIN32) && _MSC_VER >= 1700

          std::string rv(fmt);

        try
        {
	        std::regex reg(pat);

	        std::smatch m;
	        while (std::regex_search (rv,m,reg))
	        {
		        std::string mac = m.str();
		        mac = StringUtils::replaceAll(mac, rep.substr(0,1), rep.substr(1));

		        char buffer[1024];
		        sprintf(buffer, mac.c_str(), to);

		        rv.replace(m.position(), m.length(), std::string(buffer));
	        }
        }
        catch(std::regex_error& e)
        {
	        Exception E(std::string("std::regex_error:")+e.what());
	        GPSTK_THROW(E);
        }

        return rv;
#else
         regex_t re;
         const size_t bufferSize = 513;
         char buffer[bufferSize];
         int rc = regcomp(&re, pat.c_str(), REG_EXTENDED);
            // if the regex doesnt compile, toast =)
         if ( rc != 0)
         {
            regerror(rc, NULL, buffer, bufferSize - 1);
            regfree(&re);
            StringException se("Regexp error: " + std::string(buffer));
            GPSTK_THROW(se);
         }

         regmatch_t r;
         std::string rv = fmt;

         while ( regexec(&re, rv.c_str(), 1, &r, 0) == 0 )
         {
            size_t len = r.rm_eo - r.rm_so;
            std::string mac = rv.substr(r.rm_so, len);
            mac = replaceAll(mac, rep.substr(0,1), rep.substr(1));
            sprintf(buffer, mac.c_str(), to);
            rv.replace(r.rm_so, len, std::string(buffer));
         }

         regfree(&re);
         return rv;
#endif
      }

      inline std::string subString(const std::string& s,
                                   const std::string::size_type startPos,
                                   const std::string::size_type length,
                                   const char pad)
         throw(StringException)
      {
         try
         {
            if(startPos >= s.length())
            {
               return std::string(length, pad);
            }
            std::string temp = s.substr(startPos, length);
            return leftJustify(temp, length, pad);
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string& lowerCase(std::string& s)
      {
         for(std::string::size_type i = 0; i < s.length(); i++)
         {
            s[i] = tolower(s[i]);
         }
         return s;
      }

      inline std::string& upperCase(std::string& s)
      {
         for(std::string::size_type i = 0; i < s.length(); i++)
         {
            s[i] = toupper(s[i]);
         }
         return s;
      }

      inline std::string memToString(const void* p,
                                const std::string::size_type size)
      {
         unsigned char* q = (unsigned char*)p;
         std::string s(size,'\0');
         for (int i=0; i<(int)size; i++)
         {
            s[i] = (unsigned char)(*q++);
         }
         return s;
      }

      inline std::string firstWord(const std::string& s,
                              const char delimiter)
         throw(StringException)
      {
         try
         {
               // return s if there are no delimiters
            std::string::size_type pos = s.find_first_not_of(delimiter);
            if (pos == std::string::npos)
            {
               return s;
            }
               // find the end delimiter (if any) and return the string
            std::string::size_type endPos = s.find(delimiter, pos);
            if (endPos == std::string::npos)
            {
               return std::string(s, pos, endPos);
            }
            else
            {
               return std::string(s, pos, endPos - pos);
            }
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline int numWords(const std::string& s,
                          const char delimiter)
         throw(StringException)
      {
         try
         {
            std::string t(s);
            stripTrailing(t, delimiter);

            int words = 0;
            while(t.length())
            {
               stripLeading(t, delimiter);
               stripLeading(t, firstWord(t, delimiter));
               words++;
            }
            return words;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string words(const std::string& s,
                          const std::string::size_type firstWord,
                          const std::string::size_type numWords,
                          const char delimiter)
         throw(StringException)
      {
         try
         {
            if ((firstWord == 0) && (numWords == 1))
               return StringUtils::firstWord(s, delimiter);
            if (numWords == 0)
               return "";
            std::string::size_type wordNum = 0;
            std::string::size_type pos = 0, startPos = 0;

            std::string toReturn;

               // get position of word wordNum
            pos = s.find_first_not_of(delimiter, pos);
            while ((pos != std::string::npos) && (pos <= s.length()))
            {
               if (wordNum == firstWord)
                  startPos = pos;
                  // get first delimter after word wordNum
               pos = s.find(delimiter, pos);
               if (((int)numWords != -1) && ((int)wordNum == (int)(firstWord + (numWords-1))))
                  break;
               pos = s.find_first_not_of(delimiter, pos);
               wordNum++;
            }

            if (pos == std::string::npos)
               return s.substr(startPos);
            return s.substr(startPos, pos-startPos);
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string stripFirstWord(std::string& s,
                                   const char delimiter)
         throw(StringException)
      {
         try
         {
            stripLeading(s, delimiter);
            std::string toReturn = firstWord(s, delimiter);
            stripLeading(s, toReturn);
            stripLeading(s, delimiter);
            return toReturn;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::vector<std::string> split(const std::string& str,
                                            const char delimiter)
         throw(StringException)
      {
         try {
            std::vector<std::string> rvec;   // vector to return
            std::string tempStr(str);        // copy the input string
            stripLeading(tempStr,delimiter); // remove leading delimiters
            while(tempStr.size() > 0)
               rvec.push_back(stripFirstWord(tempStr,delimiter));
            return rvec;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string& removeWords(std::string& s,
                                 const std::string::size_type first,
                                 const std::string::size_type wordsToReplace,
                                 const char delimiter)
         throw(StringException)
      {
         try
         {
            std::string temp(s);
            std::string::size_type thisWord;

               // empty out s.  add the new parts of s as they are parsed
            s.erase(0, std::string::npos);

               // copy the part of the string through word 'first'
               // by appending any delimiters then appending
               // a word for however many words we're keeping.
            for(thisWord = 0; thisWord < first; thisWord++)
            {
               s.append(temp.find_first_not_of(delimiter),delimiter);
               stripLeading(temp, delimiter);
               s.append(firstWord(temp));
               stripLeading(temp, firstWord(temp));
            }

               // skip over the number of words to replace, making
               // sure to stop when there's no more string left
               // to skip
            for(thisWord = 0;
                (thisWord < wordsToReplace) &&
                   (temp.length() != 0);
                thisWord++)
            {
               stripLeading(temp, delimiter);
               stripLeading(temp, firstWord(temp));
            }

               // add on any extra words at the end
            s.append(temp);

            return s;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string doub2sci(const double& d,
                             const std::string::size_type length,
                             const std::string::size_type expLen,
                             const bool showSign,
              const bool checkSwitch)
      {
         std::string toReturn;
         short exponentLength = expLen;

            /* Validate the assumptions regarding the input arguments */
         if (exponentLength < 0) exponentLength = 1;
         if (exponentLength > 3 && checkSwitch) exponentLength = 3;

         std::stringstream c;
         c.setf(std::ios::scientific, std::ios::floatfield);

            // length - 3 for special characters ('.', 'e', '+' or '-')
            // - exponentlength (e04)
            // - 1 for the digit before the decimal (2.)
            // and if showSign == true,
            //    an extra -1 for '-' or ' ' if it's positive or negative
         int expSize = 0;
         if (showSign)
            expSize = 1;
         c.precision(length - 3 - exponentLength - 1 - expSize);


         c << d;

         c >> toReturn;

         return toReturn;
      }

      inline std::string doubleToScientific(const double& d,
                                            const std::string::size_type length,
                                            const std::string::size_type precision,
                                            const std::string::size_type explen,
                                            bool showPlus)
      {
         // get final exp length, precision and total length
         std::string::size_type elen = (explen > 0 ? (explen < 3 ? explen : 3) : 1);
         std::string::size_type prec = (precision > 0 ? precision : 1);
         std::string::size_type leng = (length > 0 ? length : 1);

         // i will be minimum length required with prec==1: force leng if necessary
         size_t i = (int(leng) - int(elen) - 4);
         if(showPlus) i--;
         if(i > 0 && leng < i) leng = std::string::size_type(i);

         // set up the stream for writing
         std::stringstream ss;
         ss << std::scientific << std::setprecision(prec);
         if(showPlus) ss << std::showpos;

         // write d to a string with precision, sign and in scientific notation
         ss << d;

         // now read that string
         std::string str1,str2;
         ss >> str1;
         std::string::size_type pos = str1.find_first_of("EDed");    // find exponent
         str2 = str1.substr(0,pos+2);        // str2 = +123.2345e+
         str1 = str1.substr(pos+2);          // str1 = exponent only

         // make the exponent length elen
         str2 += StringUtils::rightJustify(
                     StringUtils::asString(StringUtils::asInt(str1)),elen,'0');

         // pad if necessary
         if(str2.length() < leng) str2 = StringUtils::rightJustify(str2,leng);

         return str2;
      }

      inline std::string& sci2for(std::string& aStr,
                             const std::string::size_type startPos,
                             const std::string::size_type length,
                             const std::string::size_type expLen,
              const bool checkSwitch)
         throw(StringException)
      {
         try
         {
            std::string::size_type idx = aStr.find('.', startPos);
            int expAdd = 0;
            std::string exp;
            long iexp;
         //If checkSwitch is false, always redo the exponential. Otherwise,
         //set it to false.
       bool redoexp=!checkSwitch;

               // Check for decimal place within specified boundaries
            if ((idx == 0) || (idx >= (startPos + length - expLen - 1)))
            {
               StringException e("sci2for: no decimal point in string");
               GPSTK_THROW(e);
            }

               // Here, account for the possibility that there are
               // no numbers to the left of the decimal, but do not
               // account for the possibility of non-scientific
               // notation (more than one digit to the left of the
               // decimal)
            if (idx > startPos)
            {
               redoexp = true;
                  // Swap digit and decimal.
               aStr[idx] = aStr[idx-1];
               aStr[idx-1] = '.';
                  // Only add one to the exponent if the number is non-zero
               if (asDouble(aStr.substr(startPos, length)) != 0.0)
                  expAdd = 1;
            }

            idx = aStr.find('e', startPos);
            if (idx == std::string::npos)
            {
               idx = aStr.find('E', startPos);
               if (idx == std::string::npos)
               {
                  StringException e("sci2for:no 'e' or 'E' in string");
                  GPSTK_THROW(e);
               }
            }
               // Change the exponent character to D normally, or E of checkSwitch is false.
       if (checkSwitch)
               aStr[idx] = 'D';
       else
               aStr[idx] = 'E';

          // Change the exponent itself
            if (redoexp)
            {
               exp = aStr.substr(idx + 1, std::string::npos);
               iexp = asInt(exp);
               iexp += expAdd;

               aStr.erase(idx + 1);
               if (iexp < 0)
               {
                  aStr += "-";
                  iexp -= iexp*2;
               }
               else
                  aStr += "+";
               aStr += rightJustify(asString(iexp),expLen,'0');

            }

               // if the number is positive, append a space
               // (if it's negative, there's a leading '-'
            if (aStr[0] == '.')
            {
               aStr.insert((std::string::size_type)0, 1, ' ');
            }

          //If checkSwitch is false, add on one leading zero to the string
       if (!checkSwitch)
       {
          aStr.insert((std::string::size_type)1, 1, '0');
            }


            return aStr;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }  // end sci2for


      inline std::string doub2for(const double& d,
                             const std::string::size_type length,
                             const std::string::size_type expLen,
              const bool checkSwitch)
         throw(StringException)
      {
         try
         {
            short exponentLength = expLen;

               /* Validate the assumptions regarding the input arguments */
            if (exponentLength < 0) exponentLength = 1;
            if (exponentLength > 3 && checkSwitch) exponentLength = 3;

            std::string toReturn = doub2sci(d, length, exponentLength, true, checkSwitch);
            sci2for(toReturn, 0, length, exponentLength, checkSwitch);

            return toReturn;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }


      inline double for2doub(const std::string& aStr,
                             const std::string::size_type startPos,
                             const std::string::size_type length)
      {
         std::string s(aStr, startPos, length);
         strip(s);

            // you can blame Rinex for these special checks
         if (s.empty())
         {
            return 0;
         }

         std::string::size_type pos = s.find_first_of("EDd");
         if (pos != std::string::npos)
         {
            s[pos] = 'e';
         }
         else
         {
               // just treat it like a double
            return asDouble(aStr.substr(startPos, length));
         }

         std::stringstream st;
         st << s;

         double d;
         st >> d;

         return d;
      }

      inline std::string printable(const std::string& aStr)
         throw(StringException)
      {
         try
         {
            std::string rv(aStr);

            for (int i = 0; i < (int)rv.length(); i++)
            {
               char c = rv[i];
               if (!isprint(c))
               {
                  if (iscntrl(c))
                  {
                     rv.replace(i,1,2,'^');
                     rv.replace(i+1,1,1, 64+(c));
                  }
                  else
                  {
                     std::string mess(c2x(rv.substr(i,1)));
                     rv.replace(i,1,"<"+mess+">");
                  }
               }
            }

            return rv;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

      inline std::string& prettyPrint(std::string& aStr,
                                      const std::string& lineDelim,
                                      const std::string& indent,
                                      const std::string& firstIndent,
                                      const std::string::size_type len,
                                      const char wordDelim)
         throw(StringException)
      {
         try
         {
               // chop aStr up into words based on wordDelim
            std::list<std::string> wordList;
            std::string tempStr(aStr);
            stripLeading(tempStr, wordDelim);
            while (!tempStr.empty())
            {
               std::string theFirstWord = firstWord(tempStr,wordDelim);
               wordList.push_back(theFirstWord);
               stripLeading(tempStr, theFirstWord);
               stripLeading(tempStr, wordDelim);
            }

               // now reassemble the words into sentences
            std::string toReturn;
            std::string thisLine = firstIndent, lastLine;
            while (!wordList.empty())
            {
               lastLine = thisLine;
               if (!lastLine.empty())
                  thisLine += wordDelim;
               thisLine += wordList.front();

               if (thisLine.length() > len)
               {
                     // if the first word is longer than a line, just add it.
                     // if this is the first line, remember to add the indent.
                  if (lastLine.empty())
                  {
                     if (toReturn.empty())
                        lastLine += firstIndent;
                     lastLine = wordList.front();
                  }

                  toReturn += lastLine + lineDelim;

                  thisLine.erase();
                  lastLine.erase();

                  thisLine = indent;
               }
               else
                  wordList.erase(wordList.begin());
            }
            if (!thisLine.empty())
               toReturn += (thisLine + lineDelim);

            aStr = toReturn;
            return aStr;
         }
         catch(StringException &e)
         {
            GPSTK_RETHROW(e);
         }
         catch(std::exception &e)
         {
            StringException strexc("Exception thrown: " + std::string(e.what()));
            GPSTK_THROW(strexc);
         }
      }

         //@}

   } // namespace StringUtils

} // namespace gpstk
#endif // GPSTK_STRINGUTILS_HPP
