#pragma ident "$Id$"



/**
 * @file CommandOptionWithPositionArg.hpp
 * Command line options with position (class Position) arguments.
 */

#ifndef COMMANDOPTIONWITHPOSITIONARG_HPP
#define COMMANDOPTIONWITHPOSITIONARG_HPP

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






#include "CommandOption.hpp"
#include "Position.hpp"

namespace gpstk
{
      /** @addtogroup commandoptiongroup */
      //@{
   
      /** @ingroup positiongroup
       * @brief Command-line option class for processing position strings.
       * This class allows the programmer to add command-line options
       * to an application that can parse strings containing representations
       * of position.  The programmer must specify the format to be 
       * accepted.  Refer to Position::printf() for details on the formatting
       * specifications.
       */
   class CommandOptionWithPositionArg : public gpstk::CommandOptionWithAnyArg
   {
   public:
         /** Constructor
          * @param shOpt The one character command line option.  Set to 0
          *    if unused.
          * @param loOpt The long command option.  Set to std::string() 
          *    if unused.
          * @param posFormat format for scanning argument into a Position
          *    (\see Position::setToString() for details).
          * @param desc A string describing what this option does.
          * @param required Set to true if this is a required option.
          */
      CommandOptionWithPositionArg(const char shOpt,
                                   const std::string& loOpt,
                                   const std::string& posFormat,
                                   const std::string& desc,
                                   const bool required = false)
            : gpstk::CommandOptionWithAnyArg(shOpt, loOpt, desc, required),
              posSpec(posFormat)
      {}

         /// Destructor
      virtual ~CommandOptionWithPositionArg()
      {}
      
         /** Returns a string with the argument format (just "POSITION", 
          * not scanning format).
          */
      virtual std::string getArgString() const
      { return "POSITION"; }
      
         /// Validate arguments passed using this option (and store them).
      virtual std::string checkArguments();
      
         /// Return the positions scanned in from the command line.
      std::vector<Position> getPosition() const
      { return positions; }
      
   protected:
         /// Collection of positions scanned in from the command line.
      std::vector<Position> positions;
      
         /// Format used to scan positions in.
      std::string posSpec;
      
         /// Default Constructor
      CommandOptionWithPositionArg() 
      {}
      
         /// Return the appropriate position scanning format for value[index].
      virtual std::string 
      getPositionSpec(std::vector<std::string>::size_type index) const
      { return posSpec; }
      
   }; // end of class CommandOptionWithPositionArg

      //@}

}

#endif // COMMANDOPTIONWITHPOSITIONARG_HPP
