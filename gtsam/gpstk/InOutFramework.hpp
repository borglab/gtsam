#pragma ident "$Id$"

/**
 *  @file InOutFramework.hpp
 *  An extension of the looped framework that takes care of setting up the
 *  input and output streams.
 */

#ifndef INOUTFRAMEWORK_HPP
#define INOUTFRAMEWORK_HPP

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


#include "LoopedFramework.hpp"


namespace gpstk
{

      /** @addtogroup appframegroup */
      //@{

      /**
       * This is a framework for programs that take a single type of
       * input data and output a single stream of output.
       *
       * The end user should define subclasses of this class,
       * implementing those methods described as being meant to be
       * overridden.
       *
       * In use, the user will construct an object of the class
       * derived from this, then call the initialize() and run()
       * methods in that order.
       */
   template<class IType, class OType>
   class InOutFramework : public LoopedFramework
   {
   public:


      /** Constructor for InOutFramework.
       *
       * @param applName   name of the program (argv[0]).
       * @param applDesc   text description of program's function
       *                   (used by CommandOption help).
       */
      InOutFramework( const std::string& applName,
                      const std::string& applDesc )
         throw()
         : LoopedFramework(applName, applDesc)
      {};


      /// Destructor
      virtual ~InOutFramework() {};


      bool initialize( int argc,
                       char *argv[],
                       bool pretty = true )
         throw()
      {
         using std::ios;

         CommandOptionWithAnyArg
            inputOpt('i', "input",
                     "A file to take the input from. The default is stdin."),
            outputOpt('o', "output",
                      "A file to receive the output. The default is stdout.");

         if (!LoopedFramework::initialize(argc, argv, pretty))
            return false;

         if (inputOpt.getCount())
            inputFn = inputOpt.getValue()[0];


         if (inputFn=="-" || inputFn=="")
         {
            input.copyfmt(std::cin);
            input.clear(std::cin.rdstate());
            input.ios::rdbuf(std::cin.rdbuf());
            inputFn = "<stdin>";
         }
         else
         {
            input.open(inputFn.c_str(), std::ios::in);
         }


         if (!input)
         {
            std::cerr << "Could not open: " << inputFn << std::endl;
            return false;
         }

         if (outputOpt.getCount())
            outputFn = outputOpt.getValue()[0];

         if (outputFn=="-" || outputFn=="")
         {
            output.copyfmt(std::cout);
            output.clear(std::cout.rdstate());
            output.ios::rdbuf(std::cout.rdbuf());
            outputFn = "<stdout>";
         }
         else
         {
            output.open(outputFn.c_str(), std::ios::out);
         }

         if (!output)
         {
            std::cerr << "Could not open: " << outputFn << std::endl;
            return false;
         }

         if (debugLevel)
            std::cout << "Sending output to " << outputFn << std::endl
                      << "Reading input from " << inputFn << std::endl;

         return true;
      }  // End of method 'InOutFramework::initialize()'


      IType input;

      OType output;

      std::string inputFn, outputFn;


   private:


      // Do not allow the use of the default constructor.
      InOutFramework();


   }; // End of class 'InOutFramework'

      //@}

}  // End of namespace gpstk
#endif   // INOUTFRAMEWORK_HPP
