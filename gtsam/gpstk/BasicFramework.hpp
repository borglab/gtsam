#pragma ident "$Id$"

/**
 * @file BasicFramework.hpp
 * Basic framework for programs in the GPS toolkit
 */

#ifndef GPSTK_BASICFRAMEWORK_HPP
#define GPSTK_BASICFRAMEWORK_HPP

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


#include "CommandOptionParser.hpp"
#include "MainAdapter.hpp"

namespace gpstk
{

      /** @defgroup appframegroup Framework for Applications
       *
       * The application frameworks provide a set of classes that
       * perform the basic functions of applications within the GPS
       * toolkit.  That is, they provide a framework for applications
       * so that the applications only have to implement those
       * features which are unique to that application.
       *
       * The classes are defined in a tree of increasing capability,
       * that is, the BasicFramework class at the root of the tree
       * does very little and implements only those functions which
       * are common to all programs within the toolkit.  Each
       * subsequent subclass adds additional layers to these basic
       * capabilities.
       *
       * The end user is expected to create a class, which inherits
       * from one of these frameworks, and override the appropriate
       * methods in order to perform the necessary function of that
       * program.  The methods to be overridden depend on the
       * framework being used and what the program is intended to
       * do.
       */

      //@{

      /**
       * This is a (very) basic framework for programs in the GPS
       * toolkit.  It is meant to be used by programs that start up,
       * do some processing, and quit.
       *
       * The end user should define subclasses of this class,
       * implementing those methods described as being meant to be
       * overridden; initialize(), additionalSetup(), spinUp(), process(),
       * and shutDown().
       *
       * In use, the user will construct an object of the class
       * derived from this, then call the run() method.
       */
   class BasicFramework
   {
   public:


         /** Constructor for BasicFramework.
          *
          * @param applName   name of the program (argv[0]).
          * @param applDesc   text description of program's function
          *                   (used by CommandOption help).
          */
      BasicFramework( const std::string& applName,
                      const std::string& applDesc )
         throw();


         /// Destructor.
      virtual ~BasicFramework() {};


         /** Process command line arguments. When this method is overridden,
          *  make sure to call the parent class's initialize().
          *
          * @param argc    same as main() argc.
          * @param argv    same as main() argv.
          * @param pretty  Whether the 'pretty print' option will be used when
          *                printing descriptions. It is 'TRUE' by default.
          *
          * @return true if normal processing should proceed (i.e. no
          *         command line errors or help requests).
          */
      virtual bool initialize( int argc,
                               char *argv[],
                               bool pretty = true )
         throw();


         /** Process command line arguments. When this method is overridden,
          *  make sure to call the parent class's initialize().
          *
          * @param cmdLine command line(command with arguments)
          * @param pretty  Whether the 'pretty print' option will be used when
          *                printing descriptions. It is 'TRUE' by default.
          *
          * @return true if normal processing should proceed (i.e. no
          *         command line errors or help requests).
          */
      virtual bool initialize( std::string cmdLine,
                               bool pretty = true )
         throw();


         /** Run the program. Processes only once (refer to subclasses
          *  for looped processing).
          *
          * @return false if an exception occurred
          */
      bool run() throw();


   protected:

      int debugLevel;           ///< Debug level for this run of the program.
      int verboseLevel;         ///< Verbose level for this run of the program.
      std::string argv0;        ///< Name of the program.
      std::string appDesc;      ///< Description of program's function.

         /// Command-line options.
      //@{
      CommandOptionNoArg debugOption; ///< Enable debugging output and syslog message cloning to stdout.
      CommandOptionNoArg verboseOption;
      CommandOptionNoArg helpOption;
      //@}


         /**
          * Called by the run() method; calls additionalSetup(),
          * spinUp(), and process(), in that order.  Generally should
          * not be overridden.
          */
      virtual void completeProcessing();


         /**
          * Additional set-up to be performed before starting
          * processing.  This generally involves things that are
          * necessary for either the spinUp processing or main
          * processing. This method should be implemeneted by the end-user.
          */
      virtual void additionalSetup() {};


         /**
          * Code to be executed AFTER initialize() and additionalSetup().
          * This method should be implemeneted by the end-user.
          */
      virtual void spinUp() {};


         /**
          * Processing to be performed.  This method should be
          * implemeneted by the end-user.
          */
      virtual void process() {};


         /**
          * Clean-up processing to be done before the program ends.
          * This method is executed outside of a try block and should
          * be implemeneted by the end-user.
          */
      virtual void shutDown() {};


   private:


         // Do not allow the use of the default constructor.
      BasicFramework();

   }; // End of class 'BasicFramework'

      //@}

}  // End of namespace gpstk
#endif   // GPSTK_BASICFRAMEWORK_HPP
