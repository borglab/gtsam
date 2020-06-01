#pragma ident "$Id$"

/**
 * @file BasicFramework.cpp
 * Basic framework for programs in the GPS toolkit
 */

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


#include "Exception.hpp"
#include "BasicFramework.hpp"
#include "StringUtils.hpp"


namespace gpstk
{

   using namespace std;


   BasicFramework :: BasicFramework( const string& applName,
                                     const string& applDesc )
      throw()
         : debugLevel(0),
           verboseLevel(0),
           argv0(applName),
           appDesc(applDesc),
           debugOption('d', "debug", "Increase debug level"),
           verboseOption('v', "verbose", "Increase verbosity"),
           helpOption('h', "help", "Print help usage")
   {} // End of constructor 'BasicFramework::BasicFramework()'



   bool BasicFramework :: initialize( int argc,
                                      char *argv[],
                                      bool pretty )
      throw()
   {

         // Creating the parser here ensures that all the subclasses'
         // option objects are constructed.
      CommandOptionParser cop(appDesc);

      cop.parseOptions(argc, argv);

      if (helpOption.getCount())
      {
         cop.displayUsage(cerr, pretty);
         return false;
      }

      if (cop.hasErrors())
      {
         cop.dumpErrors(cerr);
         cop.displayUsage(cerr, pretty);
         return false;
      }

      debugLevel = debugOption.getCount();
      verboseLevel = verboseOption.getCount();

      return true;

   }  // End of method 'BasicFramework::initialize()'

       
   bool BasicFramework :: initialize( std::string cmdLine,
                                      bool pretty )
      throw()
   {
      std::vector<std::string> vArgs;
      vArgs.clear();

      std::string cmd(cmdLine);
      while(cmd.length())
      {
         vArgs.push_back(StringUtils::stripFirstWord(cmd));
      }

      int argc = vArgs.size();
      char** argv = new char*[argc];
      if(!argv)
      {
         return false;
      }

      for(int i=0; i<argc; i++)
      {
         argv[i] = &vArgs[i][0];
      }

      bool state = initialize(argc, argv, pretty);

      // delete memory *char[]
      delete[] argv;

      return state;

   }  // End of method 'BasicFramework::initialize()'


   bool BasicFramework :: run()
      throw()
   {

      try
      {
         completeProcessing();
      }
      catch (Exception& exc)
      {
         cerr << exc;
         return false;
      }
      catch (...)
      {
         cerr << "Caught unknown exception" << endl;
         return false;
      }

      shutDown();

      return true;

   }  // End of method 'BasicFramework::run()'



   void BasicFramework :: completeProcessing()
   {
      additionalSetup();

      spinUp();

      process();

   }  // End of method 'BasicFramework::completeProcessing()'


}  // End of namespace gpstk
