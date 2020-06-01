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
 * @file CommandOptionParser.cpp
 * Parse command line options.
 */

#include <cstring>
#include "CommandOptionParser.hpp"

#include "StringUtils.hpp"

using namespace std;
using namespace gpstk::StringUtils;

#ifdef _MSC_VER
   #if ( _MSC_VER < 1600 )
   #define min(VAL1, VAL2) _cpp_min(VAL1, VAL2)
   #define max(VAL1, VAL2) _cpp_max(VAL1, VAL2)
   #endif
#endif

namespace gpstk
{

      // parses the command line input
   void
   CommandOptionParser::parseOptions(int argc,
                                     char* argv[])
   {
         // this maps the index of optionVec to the command line options
      CommandOptionMap com;
      
         // keep track of the order of command options
      unsigned int order = 0;

         // get the program name.  If there's a / in it, take the part
         // after the last / as the program name
      progName = string(argv[0]);
      string::size_type slashPos = progName.rfind('/');
      if (slashPos != string::npos)
         progName = progName.substr(slashPos + 1);

      string shortOptString;
      struct option* optArray = NULL;
      unsigned long optArraySize = 0;

      CommandOption *trailing = NULL;

         // build the getopt and getopt_long inputs
      CommandOptionVec::size_type index;
      for(index = 0; index < optionVec.size(); index++)
      {
            // add short options
         switch (optionVec[index]->optType)
         {
            case CommandOption::trailingType:
               if (trailing)
                  errorStrings.push_back("More than one trailing argument"
                                         " object used (programming error");
               else
                  trailing = optionVec[index];
               break;
            case CommandOption::stdType:
               if (optionVec[index]->shortOpt != 0)
               {
                  shortOptString += optionVec[index]->toGetoptShortOption();
                  com[string(1,optionVec[index]->shortOpt)] = optionVec[index];
               }

                  // add long options
               if (!optionVec[index]->longOpt.empty())
               {
                  resizeOptionArray(optArray, optArraySize);
                  optArray[optArraySize - 1] = 
                     optionVec[index]->toGetoptLongOption();
                  com[optionVec[index]->longOpt] = optionVec[index];
               }

                  // keep track of whether or not there are required or
                  // optional arguments
               if (optionVec[index]->required)
                  hasRequiredArguments = true;
               else
                  hasOptionalArguments = true;
               break;
            default:
                  // do nothing
               break;
         }
      }

         // add the getopt_long terminator value
      resizeOptionArray(optArray, optArraySize);
      struct option lastOption = {0,0,0,0};
      optArray[optArraySize - 1] = lastOption;

         // use '+' to make getopt not mangle the inputs (if i remember right)
      shortOptString.insert((string::size_type)0, (string::size_type)1, '+');

      int cha;
      int optionIndex;

         // disable internal error messages
      opterr = 0;

      while (optind < argc)
      {
         if ((cha = getopt_long(argc, argv, shortOptString.c_str(),
                                optArray, &optionIndex)) == -1)
         {
            if (!trailing)
               errorStrings.push_back("Excess arguments");
            break;
         }

         order++;

            // Solaris uses '?' for all getopt errors.  Linux uses '?'
            // for unknown options and ':' for options that require
            // arguments but don't have then. That's why the error
            // message is "option error" cause we can't differentiate
            // what the REAL error is...
         if ((cha == '?') || (cha == ':'))
         {
               // get the argument that had the error and write an
               // error string
            string errorArg;
               // for a character option error
            if (optopt != 0)
               errorArg = string(1, (char)optopt);
               // for a getopt_long error
            else
               errorArg = argv[optind - 1];
            errorStrings.push_back(string("Option error: " + errorArg));
         }
            // otherwise this is probably a found option
         else
         {
            string thisOption;

               // determine whether it found the short or long version
            if (cha != 0)
               thisOption = string(1,(char)cha);
            else
               thisOption = string(optArray[optionIndex].name);
               
               // try to find the option in our option map
            map<string, CommandOption*>::iterator itr = com.find(thisOption);

            if (itr != com.end())
            {
               CommandOption* pickedOption = (*itr).second;
                  // if there is an value for this option...
               if (optarg)
               {
                  if (pickedOption->optFlag == CommandOption::noArgument)
                  {
                     errorStrings.push_back(string("Option ") +
                                            thisOption +
                                            string(" has an argument but it shouldn't."));
                  }
                     // add this argument to the picked option and
                     // increment the count
                  else if (pickedOption->optFlag == CommandOption::hasArgument)
                  {
                     pickedOption->value.push_back(string(optarg));
                     pickedOption->count++;
                     pickedOption->order = order;
                  }

               }
                  // no value for option...
               else
               {
                  if (pickedOption->optFlag == CommandOption::hasArgument)
                  {
                     errorStrings.push_back(string("Option ") +
                                            thisOption +
                                            string(" has no argument when it should."));
                  }
                     // increment the picked option's count
                  else if (pickedOption->optFlag == CommandOption::noArgument)
                  {
                     pickedOption->count++;
                     pickedOption->order = order;
                  }
               }
            } // itr != end()
            else
            {
               errorStrings.push_back("Unknown option error");               
            }
         } // else cha ==
      }  // getopt_long

         // check for remaining arguments
      if (optind < argc)
      {
         if (trailing)
         {
            int i;
            for(i = optind; i < argc; i++)
            {
               trailing->value.push_back(string(argv[i]));
               trailing->count++;
            }
         }
            // the case where trailing==null is handled above
      }

      for(index = 0; index < optionVec.size(); index++)
      {
         string retVal = optionVec[index]->checkArguments();
         if (!retVal.empty())
            errorStrings.push_back(retVal);

            // check max count
         if (optionVec[index]->maxCount != 0)
         {
            if (optionVec[index]->count > optionVec[index]->maxCount)
            {
               string errstr("Option ");
               errstr += optionVec[index]->getOptionString();
               errstr += string(" appeared more times than allowed.");
               errorStrings.push_back(errstr);
            }
         }
      }
   
      delete [] optArray;
   }

   ostream& CommandOptionParser::dumpErrors(ostream& out)
   {
      vector<string>::size_type index;
      for(index = 0; index < errorStrings.size(); index++)
         out << errorStrings[index] << endl;
      return out;
   }

      // prints the required arguments first (if any) then the optional
      // ones (if any)
   ostream& CommandOptionParser::displayUsage(ostream& out, bool doPretty)
   {
      CommandOptionVec::size_type index;
      CommandOption *trailing = NULL;

      char *colch = getenv("COLUMNS");
      int columns = 80;
      unsigned maxlen = 0;
      if (colch)
      {
         string colStr(colch);
         columns = asInt(colStr);
      }

         // find the trailing argument if any, and max option string length
      for (index = 0; index < optionVec.size(); index++)
      {
         if (optionVec[index]->optType == CommandOption::trailingType)
            trailing = optionVec[index];
         else if (optionVec[index]->optType == CommandOption::stdType)
            maxlen = std::max(maxlen,
               unsigned(optionVec[index]->getFullOptionString().length()));
      }

      out << "Usage: " << progName;
      if (hasRequiredArguments || hasOptionalArguments)
         out << " [OPTION] ...";
      if (trailing)
         out << " " << trailing->description;
      out << endl
          << (doPretty ? prettyPrint(text,"\n","","",columns) : text);
// << endl
//          << endl 
//          << "Command options:" << endl;
      
      for(int required = 1; required >= 0; required--)
      {
         if (required==1 && hasRequiredArguments)
            out << endl << "Required arguments:" << endl;
         else if (required==0 && hasOptionalArguments)
            out << endl << "Optional arguments:" << endl;

         for(index = 0; index < optionVec.size(); index++)
         {
            if ((optionVec[index]->required == (required==1)) &&
                (optionVec[index]->optType == CommandOption::stdType))
            {
               string optstr(optionVec[index]->getFullOptionString());
               string desc(optionVec[index]->description);
               string indent(maxlen, ' ');

               if(doPretty) {
                  leftJustify(optstr, maxlen);
                  prettyPrint(desc, "\n", indent, optstr, columns);
               }
               out << desc;
               if(!doPretty) out << endl;
            }
         }
      }

      return out;
   }

      // resizes the array for getopt_long
   void CommandOptionParser::resizeOptionArray(struct option *&oldArray,
                                               unsigned long& oldSize)
   {
      struct option* newArray = new struct option[1 + oldSize];
      std::memcpy(newArray, oldArray, oldSize * sizeof(struct option));
      delete [] oldArray;
      oldArray = newArray;
      newArray = NULL;
      oldSize += 1;
   }

}  // end namespace gpstk
