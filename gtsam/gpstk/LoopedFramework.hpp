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

#ifndef GPSTK_LOOPEDFRAMEWORK_HPP
#define GPSTK_LOOPEDFRAMEWORK_HPP

/**
 *  @file LoopedFramework.hpp
 *  Basic framework for programs processing loops in the GPS toolkit
 */

#include "BasicFramework.hpp"

namespace gpstk
{
   /** @addtogroup appframegroup */
   //@{

   /**
    * This is a basic framework for programs processing in loops in
    * the GPSTK.
    *
    * The end user should define subclasses of this class,
    * implementing those methods described as being meant to be
    * overridden; initialize(), additionalSetup(), spinUp(), process(), and
    * shutDown().
    * In the process() method, simply set variable timeToDie true prior to
    * returning for the program to call shutDown() and then terminate.
    *
    * In use, the user will construct an object of the class
    * derived from this, then call the initialize() and run()
    * methods in that order.
    */
   class LoopedFramework : public BasicFramework
   {
   public:
      /**
       * Constructor for LoopedFramework.
       * @param applName name of the program (argv[0]).
       * @param applDesc text description of program's function
       * (used by CommandOption help).
       */
      LoopedFramework(const std::string& applName,
                      const std::string& applDesc)
         throw()
         : BasicFramework(applName, applDesc), timeToDie(false)
      { }

      /// Destructor.
      virtual ~LoopedFramework() {}

   protected:
      bool timeToDie;   ///< if set to true, the loop will terminate

      /**
       * Called by the run() method, calls additionalSetup(),
       * spinUp(), and process(), in that order. Generally should not be
       * overridden.
       */
      virtual void completeProcessing();

   private:
      // Do not allow the use of the default constructor.
      LoopedFramework();
   }; // class LoopedFramework

//@}

} // namespace gpstk

#endif
