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
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software. 
//
// Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

#ifndef SINGLETON_TEMPLATE_INCLUDE
#define SINGLETON_TEMPLATE_INCLUDE

/** @file singleton.hpp  Singleton template. */

template <class T> class Singleton {
public:
   static T& Instance() {
      static T theInstance;
      return theInstance;
   }
protected:
   Singleton() {}                            // c'tor protected
   virtual ~Singleton() {}                   // d'tor virtual and protected
private:
   Singleton(Singleton const&);              // copy c'tor prohibited
   Singleton& operator=(Singleton const&);   // operator= prohibited
};
#endif   //SINGLETON_TEMPLATE_INCLUDE

/* use this class like this:

// main.cpp cf. Docs/Cpp/singleton.pdf
#include <iostream>
#include <iomanip>
#include "singleton.hpp"

class MyClass : public Singleton<MyClass> {  // Curiously Recurring Template Pattern
public:
   void set(int n) { x = n; }          // these are required only if data is private
   int get() const { return x; }       //    and you wish to control access.
   double y;                           // public data is fine.
protected:
                                       // MyClass must have empty c'tor; there are
                                       // ways to mod this - see s'ton articles.
   MyClass() : x(0),y(3.14159) { }     // set defaults here
   friend class Singleton<MyClass>;    // so Instance() can call MyClass c'tor
private:
   int x;                              // private data is fine.
};

int main(void) {
   // 'get' the singleton; this may appear anywhere MyClass is defined.
   MyClass& CFG=MyClass::Instance();

   std::cout << "x = " << CFG.get() << " y = " << CFG.y << std::endl;
   std::cout << "set x to 17 and y *= 2" << std::endl;
   CFG.set(17);
   CFG.y *= 2.0;
   std::cout << "x = " << CFG.get() << " y = " << CFG.y << std::endl;

   std::cout << "Get a new object" << std::endl;
   MyClass& newCFG=MyClass::Instance();
   std::cout << "new x = " << newCFG.get() << " y = " << newCFG.y << std::endl;
   std::cout << "set old x to 23 and new y to 37." << std::endl;
   CFG.set(23);
   newCFG.y = 37.;
   std::cout << "old x = " << CFG.get() << " y = " << CFG.y << std::endl;
   std::cout << "new x = " << newCFG.get() << " y = " << newCFG.y << std::endl;

   return 0;
}
*/
