#pragma ident "$Id$"

/**
 * @file MainAdapter.hpp
 * Easy writing programs in the GPS toolkit. 
 */

#ifndef GPSTK_MAINADAPTER_HPP
#define GPSTK_MAINADAPTER_HPP

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
//  Wei Yan - Chinese Academy of Sciences . 2009, 2010, 2011
//
//============================================================================


#include <iostream>
#include "Exception.hpp"

namespace gpstk
{

      /** @addgroup appframegroup */
       
      //@{

      /**
       * This is an assist class for the macros 'GPSTK_START_MAIN()'
       * the type T should be a subclass of the class 'BasicFramework'.
       */
   template<typename T>
   class MainAdapter
   {
   public:
      virtual int run(int argc, char* argv[])
      {
         try
         {
            T program;
            if (!program.initialize(argc, argv, true)) return 0;
            if (!program.run()) return 1;

            return 0;
         }
         catch(Exception& e)
         {
            std::cerr << "Problem: "<< e << std::endl;
         }
         catch(std::exception& e)
         {
            std::cerr << "Problem: "<< e.what() << std::endl;
         }
         catch(...)
         {
            std::cerr << "Problem: " << "Unknown error." << std::endl;
         }

         return -1;
      }

   }; // End of class 'MainAdapter'


   /// A typical way to use this class follows:
   /// 
   /// @code
   /// class PhDRTK: public BasicFramework;
   /// 
   /// GPSTK_START_MAIN(PhDRTK);
   /// @endcode
   
#define GPSTK_START_MAIN(T)         \
   int main(int argc, char* argv[]) \
   {                                \
      MainAdapter<T> M;             \
      return M.run(argc,argv);      \
   }                                \

      // @}

}  // End of namespace gpstk


#endif   // GPSTK_MAINADAPTER_HPP
