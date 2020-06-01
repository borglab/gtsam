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
 * @file CommandOption.cpp
 * Command line options
 */

#include "CommandOption.hpp"
#include "StringUtils.hpp"

#include <sstream>

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   CommandOptionVec defaultCommandOptionList;

      // Prints out short options with a leading '-' and long ones with '--'.
      // Puts a '|' between them if it has both.
   string CommandOption::getOptionString() const
   {
      string toReturn;
      if (shortOpt != 0)
      {
         toReturn += string("-") + string(1, shortOpt);
         if (!longOpt.empty())
            toReturn += string(" | --") + longOpt;
      }
      else
      {
         toReturn += string("--") + longOpt;
      }
      return toReturn;
   }

      // Prints out short options with a leading '-' and long ones with '--'.
      // Puts a ',' between them if it has both.
   string CommandOption::getFullOptionString() const
   {
      string toReturn("  ");
      if (shortOpt != 0)
      {
         toReturn += string("-") + string(1, shortOpt);
         if (!longOpt.empty())
         {
            toReturn += string(", --") + longOpt;
            if (optFlag == hasArgument)
               toReturn += "=" + getArgString();
         }
      }
      else
      {
         toReturn += string("    --") + longOpt;
         if (optFlag == hasArgument)
            toReturn += "=" + getArgString();
      }
      return toReturn;
   }

      // creates the struct option for getopt_long
   struct option CommandOption::toGetoptLongOption() const
   { 
      struct option o = {longOpt.c_str(), optFlag, NULL, 0};
      return o;
   }

      // makes the string for getopt
   std::string CommandOption::toGetoptShortOption() const
   { 
      std::string opt(1, shortOpt);
      if (optFlag == hasArgument) opt += ":";
      return opt;
   }

      // writes out the vector of values for this command option
   std::ostream& CommandOption::dumpValue(std::ostream& out) const
   {
      std::vector<std::string>::const_iterator itr = value.begin();
      while(itr != value.end()) 
      {
         out << *itr << std::endl; 
         itr++;
      }
      return out;
   }

      // returns a string like this:
      //
      //   -f | --foo  <arg>
      //        this is the description
      //
   std::string CommandOption::getDescription() const
   {
      ostringstream out;
         // do the option itself first
      out << '\t';
      if (shortOpt != 0)
      {
         out << '-' << shortOpt;
         if (!longOpt.empty())
            out << " | ";
         else
            out << '\t';        
      }
      if (! longOpt.empty())
      {
         out << "--" << longOpt;
      }
      if (optFlag == hasArgument)
      {
         out << " " << getArgString();
      }
         // and the description goes on a new line
      out << endl << prettyPrint(description, 
                                 "\n",
                                 "                  ", 
                                 "               ");
      if (maxCount != 0)
      {
         out << "\t\tUp to " << maxCount << " may be used on the command line."
             << endl;
      }
      return out.str();
   }

      // this checks if it expects number or string type arguments.
      // it returns a string describing the error, if any.
   string CommandOption::checkArguments()
   {
      if (required && (count == 0))
         return "Required option " + getOptionString() + " was not found.";

      return string();
   }

   string CommandOptionRest::checkArguments()
   {
      if (required && (count == 0))
         return "Required trailing argument was not found.";

      return string();
   }

   string CommandOptionWithNumberArg::checkArguments()
   {
      string errstr = CommandOption::checkArguments();

      if (!errstr.empty())
         return errstr;

      vector<string>::size_type vecindex;
      for(vecindex = 0; vecindex < value.size(); vecindex++)
      {
         if (!isDigitString(value[vecindex]))
         {
            string errstr("Argument for ");
            errstr += getOptionString();
            errstr += string(" should be a digit string.");
            return errstr;
         }
      }

      return string();
   }

   string CommandOptionWithStringArg::checkArguments()
   {
      string errstr = CommandOption::checkArguments();

      if (!errstr.empty())
         return errstr;

      vector<string>::size_type vecindex;
      for(vecindex = 0; vecindex < value.size(); vecindex++)
      {
         if (!isAlphaString(value[vecindex]))
         {
            string errstr("Argument for ");
            errstr += getOptionString();
            errstr += string(" should be an alphabetic string.");
            return errstr;
         }
      }
      return errstr;
   }

   string CommandOptionMutex::checkArguments()
   {
      if (doOneOfChecking)
      {
         string oo = CommandOptionOneOf::checkArguments();
         if (oo != string())
            return oo;
      }

         // mutex doesn't call CommandOption::checkArguments because
         // it uses "required" differently
      string errstr("Only one of the following options may be specified: ");
      int firstSpec = -1;
      bool touched = false;

      for (size_t i = 0; i < optionVec.size(); i++)
      {
         CommandOption *opt = optionVec[i];

         if (i)
            errstr += ", ";
         errstr += opt->getOptionString();
         if (opt->getCount())
         {
            if (firstSpec != -1)
               touched = true;
            else
               firstSpec = i;
         }
      }

      if (touched)
         return errstr;

      return string();
   }

   string CommandOptionOneOf::checkArguments()
   {
         // one-of doesn't call CommandOption::checkArguments because
         // it doesn't use "required"
      string errstr("One of the following options must be specified: ");
      bool found = false;

      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (optionVec[i]->getCount())
            found = true;
         if (i > 0)
            errstr += ", ";
         errstr += optionVec[i]->getOptionString();
      }

      if (!found)
         return errstr;

      return string();
   }

   CommandOption* CommandOptionOneOf::whichOne() const
   {
      CommandOption *rv = NULL;

      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (optionVec[i]->getCount())
         {
            rv = optionVec[i];
            break;
         }
      }

      return rv;
   }

   string CommandOptionAllOf::checkArguments()
   {
      string errstr("The following options must be used together: ");
      bool found = false, notFound = false;

      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (optionVec[i]->getCount())
            found = true;
         else
            notFound = true;
         if (i > 0)
            errstr += ", ";
         errstr += optionVec[i]->getOptionString();
      }

      if (found && notFound)
         return errstr;

      return string();
   }

   unsigned long CommandOptionAllOf::getCount() const
   {
      unsigned long rv = 0;
      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (optionVec[i]->getCount() == 0)
            return 0;
         rv += optionVec[i]->getCount();
      }
      return rv;
   }

   string CommandOptionDependent::checkArguments()
   {
         // dependent doesn't call CommandOption::checkArguments because
         // it doesn't use "required"
      string errstr;

      if (!requiree)
         errstr = "Null requiree (parent) for CommandOptionDependent";
      if (!requirer)
         errstr = "Null requirer (child) for CommandOptionDependent";

      if (requirer->getCount() && !requiree->getCount())
         errstr = "Option " + requirer->getOptionString() + " requires " +
            requiree->getOptionString();

      return errstr;
   }

   string CommandOptionGroupOr::getOptionString() const
   {
      string rv;
      if (optionVec.size() > 1)
         rv += "(";
      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (i) rv += ",";
         rv += optionVec[i]->getOptionString();
      }
      if (optionVec.size() > 1)
         rv += ")";

      return rv;
   }

   unsigned long CommandOptionGroupOr::getCount() const
   {
      unsigned long rv = 0;
      for (size_t i = 0; i < optionVec.size(); i++)
         rv += optionVec[i]->getCount();

      return rv;
   }

   unsigned long CommandOptionGroupAnd::getCount() const
   {
      unsigned long rv = 0;
      for (size_t i = 0; i < optionVec.size(); i++)
      {
         if (optionVec[i]->getCount() == 0)
            return 0;
         rv += optionVec[i]->getCount();
      }
      return rv;
   }

} // namespace gpstk
