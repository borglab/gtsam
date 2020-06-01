#pragma ident "$Id$"



/**
 * @file CommandOptionParser.hpp
 * Parse command line options
 */

#ifndef COMMANDOPTIONPARSER_HPP
#define COMMANDOPTIONPARSER_HPP

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

#include <cstring>
#include <vector>
#include <map>
#include <ostream>

namespace gpstk
{
      /** @addtogroup commandoptiongroup Command-Line Options */
      //@{

      /** 
       * This class parses the command line options and modifies the
       * corresponding CommandOptions.  By default, any CommandOptions you 
       * create will be put on a static vector<CommandOption> which is used
       * by CommandOptionParser.  You can make your own as well but that
       * isn't necessary.  You can also use addOption() to add individual
       * CommandOptions to the parser, but again this isn't necessary as the
       * default list is usually sufficient.
       *
       * Call parseOptions() to process the command line, then 
       * call hasErrors() to see if there
       * were any problems parsing the string. Errors can occur when
       * a required option isn't found on the command line, when an option
       * requiring an argument doesn't have one, or when an argument appers
       * more than its maxCount number of times among other errors.
       * If so, use dumpErrors() to
       * display the errors to an output stream, then use
       * displayUsage() to display a well formatted list of the correct
       * command line options.  Of
       * course, you can just as well ignore any command line
       * errors. After hitting an error (which most often happens when
       * it hits an argument that has no CommandOption), you can use
       * CommandOptionRest to get the unprocessed command line
       * options.
       *
       * @sa the getopttest.cpp file in the test code for some examples.
       */
   class CommandOptionParser
   {
   public:
         /// Typedef for a map between the command line option (-f) and the
         /// associated CommandOption.
      typedef std::map<std::string, gpstk::CommandOption*> CommandOptionMap;
      
         /**
          * Constructor given a text description of the program.
          * @param description a short description of this program
          * @param optList a CommandOptionVec with the list of
          *   CommandOptions for this parser.
          */
      CommandOptionParser(const std::string& description,
                          const CommandOptionVec optList = 
                          defaultCommandOptionList)
         : optionVec(optList), hasRequiredArguments(false), 
           hasOptionalArguments(false), text(description)

         {}

         /// Adds the CommandOption to the list for parsing.
      CommandOptionParser& addOption(gpstk::CommandOption& co)
         { optionVec.push_back(&co); return *this; }
      
         /// Parses the command line.
      void parseOptions(int argc,  char* argv[]);
      
         /// Returns true if any processing errors occurred.
      bool hasErrors() { return !errorStrings.empty(); }
         /// Writes the errors to \c out.
      std::ostream& dumpErrors(std::ostream& out);

         /** Writes the arguments nicely to the output.
          * @param out ostream on which to write
          * @param doPretty if true (the default), 'pretty print' descriptions
          */
      std::ostream& displayUsage(std::ostream& out, bool doPretty=true);

   private:
         /// changes the size of the option array for getopt_long.
      void resizeOptionArray(struct option* &oldArray, unsigned long& oldSize);
      
         /// The vector of CommandOptions for the parser
      CommandOptionVec optionVec;
         /// The vector of error strings for displaying to the user.
      std::vector<std::string> errorStrings;
         /// The vector of unprocessed command line arguments.
         //std::vector<std::string> remainingArguments;

         /// whether or not this command line has any rrequired options
      bool hasRequiredArguments;
         /// whether or not this command line has optional options
      bool hasOptionalArguments;

         /// the description of this program
      std::string text;

         /// the name of this program
      std::string progName;
   };
      //@}
}

#endif
