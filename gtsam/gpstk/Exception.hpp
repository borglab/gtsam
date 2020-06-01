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
 * @file Exception.hpp
 * Exceptions for all of GPSTK, including location information
 */

// The unusual include macro below is done this way because xerces
// #defines EXCEPTION_HPP in their own exception class header file.
#ifndef GPSTK_EXCEPTION_HPP
#define GPSTK_EXCEPTION_HPP

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>

namespace gpstk
{
      /**
       * @defgroup exceptiongroup Exception Classes
       * These classes are the exceptions that can be thrown in
       * the library code. Use these in your catch() blocks
       * and you'll be able to get more information
       * than what std::exception provides.  Use GPSTK_THROW()
       * and GPSTK_RETHROW() to throw or rethrow these exceptions
       * to automatically add line and file information to your
       * exceptions.
       */

      /// A class for recording locations (in the source code) of
      /// exceptions being thrown.
   class ExceptionLocation
   {
   public:
         /**
          * Constructor for location information.
          * @param filename name of source file where exception occurred.
          * @param funcName name of function where exception occurred.
          * @param lineNum line of source file where exception occurred.
          */
      ExceptionLocation(const std::string& filename = std::string(),
                        const std::string& funcName = std::string(),
                        const unsigned long& lineNum = 0)
            : fileName(filename), functionName(funcName),
              lineNumber(lineNum)
      { }

         /**
          * Destructor.
          */
      ~ExceptionLocation() {}

         /// Accessor for name of source file where exception occurred.
      std::string getFileName() const
      { return fileName; }
         /// Accessor for name of function where exception occurred.
      std::string getFunctionName() const
      { return functionName; }
         /// Accessor for line of source file where exception occurred.
      unsigned long getLineNumber() const
      { return lineNumber; }

         /**
          * Debug output function.
          * @param s stream to output debugging information for this class to.
          */
      void dump(std::ostream& s) const;

         /// Dump to a string
      std::string what() const;

         /**
          * Output stream operator for ::ExceptionLocation.
          * This is intended just to dump all the data in the
          * ::ExceptionLocation to the indicated stream.  \warning Warning: It
          * will _not_ preserve the state of the stream.
          * @param s stream to send ::ExceptionLocation information to.
          * @param e ::ExceptionLocation to "dump".
          * @return a reference to the stream \c s.
          */
      friend std::ostream& operator<<( std::ostream& s,
                                       const ExceptionLocation& e );

   private:
         /// Name of source file where exception occurred.
      std::string fileName;
         /// Name of function where exception occurred.
      std::string functionName;
         /// Line in source file where exception occurred.
      unsigned long lineNumber;
   }; // class ExceptionLocation

      /**
       * The Exception class is the base class from which all
       * exception objects thrown in the library are derived. None of
       * the functions in this class throws exceptions because an
       * exception has probably already been thrown or is about to be
       * thrown.  Each exception object contains the following:
       * -  A stack of exception message text strings (descriptions).
       * -  An error ID.
       * -  A severity code.
       * -  An error code group.
       * -  Information about where the exception was thrown.
       *
       * Exception provides all of the functions required for it and
       * its derived classes, including functions that operate on the
       * text strings in the stack.
       *
       * @sa exceptiontest.cpp for some examples of how to use this class.
       *
       * @ingroup exceptiongroup
       */
   class Exception
   {
   public:
         /// Exception severity classes.
      enum Severity
      {
         unrecoverable, /**< program can not recover from this exception */
         recoverable    /**< program can recover from this exception */
      };

         /**
          * Default constructor.
          * Does nothing.
          */
      Exception();

         /**
          * Full constructor for exception.
          * @param errorText text message detailing exception.
          * @param errorId error code related to exception e.g. MQ result
          * code.
          * @param severity severity of error.
          */
      Exception(const std::string& errorText,
                const unsigned long& errorId = 0,
                const Severity& severity = unrecoverable);


         /**
          * Full constructor for exception.
          * @param errorText text message detailing exception.
          * @param errorId error code related to exception e.g. MQ result
          * code.
          * @param severity severity of error.
          */
      Exception(const char* errorText,
                const unsigned long& errorId = 0,
                const Severity& severity = unrecoverable);


         /// Copy constructor.
      Exception(const Exception& exception);

         /// Destructor.
      ~Exception()
      {};

         /// Assignment operator.
      Exception& operator=(const Exception& e);

         /**
          * Ends the application. Normally, the library only intends
          * this function to be used internally by the library's
          * exception-handling macros when the compiler you are using
          * does not support C++ exception handling. This only occurs
          * if you define the NO_EXCEPTIONS_SUPPORT macro.
          */
      void terminate()
      { exit(1); };

         /// Returns the error ID of the exception.
      unsigned long getErrorId() const
      { return errorId; };

         /**
          * Sets the error ID to the specified value.
          * @param errId The identifier you want to associate with
          * this error.
          */
      Exception& setErrorId(const unsigned long& errId)
      { errorId = errId; return *this; };

         /**
          * Adds the location information to the exception object. The
          * library captures this information when an exception is
          * thrown or rethrown. An array of ExceptionLocation objects
          * is stored in the exception object.
          *
          * @param location An IExceptionLocation object containing
          * the following:
          * \li          Function name
          * \li          File name
          * \li          Line number where the function is called
          */
      Exception& addLocation(const ExceptionLocation& location);

         /**
          * Returns the ExceptionLocation object at the specified index.
          * @param index If the index is not valid, a 0
          * pointer is returned. (well, not really since someone
          * changed all this bah)
          */
      const ExceptionLocation getLocation(const size_t& index=0) const;

         /// Returns the number of locations stored in the exception
         /// location array.
      size_t getLocationCount() const;

         /**
          * If the thrower (that is, whatever creates the exception)
          * determines the exception is recoverable, 1 is returned. If
          * the thrower determines it is unrecoverable, 0 is returned.
          */
      bool isRecoverable() const
      { return (severity == recoverable); }

         /**
          * Sets the severity of the exception.
          * @param sever Use the enumeration Severity to specify
          * the severity of the exception.
          */
      Exception& setSeverity(const Severity& sever)
      { severity = sever; return *this; };

         /**
          * Appends the specified text to the text string on the top
          * of the exception text stack.
          * @param errorText The text you want to append.
          */
      Exception& addText(const std::string& errorText);

         /**
          * Returns an exception text string from the exception text
          * stack.
          *
          * @param index The default index is 0, which is the
          * top of the stack. If you specify an index which is not
          * valid, a 0 pointer is returned.
          */
      std::string getText(const size_t& index=0) const;

         /// Returns the number of text strings in the exception text stack.
      size_t getTextCount() const;

         /// Returns the name of the object's class.
      std::string getName() const
      { return "Exception"; };

         /**
          * Debug output function.
          * @param s stream to output debugging information for this class to.
          */
      void dump(std::ostream& s) const;

         /// Dump to a string
      std::string what() const;

         /**
          * Output stream operator for ::Exception.
          * This is intended just to dump all the data in the ::Exception to
          * the indicated stream.  \warning Warning:  It will _not_ preserve
          * the state of the stream.
          * @param s stream to send ::Exception information to.
          * @param e ::Exception to "dump".
          * @return a reference to the stream \c s.  */
      friend std::ostream& operator<<( std::ostream& s,
                                       const Exception& e );

   protected:
         /// Error code.
      unsigned long errorId;
         /// Stack of exception locations (where it was thrown).
      std::vector<ExceptionLocation> locations;
         /// Severity of exception.
      Severity severity;
         /// Text stack describing exception condition.
      std::vector<std::string> text;

         /**
          * This is the streambuf function that actually outputs the
          * data to the device.  Since all output should be done with
          * the standard ostream operators, this function should never
          * be called directly.  In the case of this class, the
          * characters to be output are stored in a buffer and added
          * to the exception text after each newline.
          */
      int overflow(int c);

   private:
         /// Buffer for stream output.
      std::string streamBuffer;
   }; // class Exception


}  // namespace gpstk


/**
 * Just a comment for the wary.  These following macros are quite
 * useful.  They just don't work under gcc 2.96/linux.  If you can fix
 * them I would be quite greatful but I am not holding my breath.  For
 * now, I am just manually putting the code where it needs to be.  The
 * problem seems to be with the __FILE__, __FUNCTION__, LINE__ being
 * defined in a macro that is in a .hpp file as opposed to the .cpp
 * file where the code gets used.  When you do it you get a segfault.
 * See the exceptiontest.cpp code in the base/test directory.
 */
#if defined ( __FUNCTION__ )
#define FILE_LOCATION gpstk::ExceptionLocation(__FILE__, __FUNCTION__, __LINE__)
#else
#define FILE_LOCATION gpstk::ExceptionLocation(__FILE__, "", __LINE__)
#endif

// For compilers without exceptions, die if you get an exception.
#if defined (NO_EXCEPTIONS_SUPPORT)
/// A macro for adding location when throwing an gpstk::Exception
/// @ingroup exceptiongroup
#define GPSTK_THROW(exc) { exc.addLocation(FILE_LOCATION); exc.terminate(); }
/// A macro for adding location when rethrowing an gpstk::Exception
/// @ingroup exceptiongroup
#define GPSTK_RETHROW(exc) { exc.addLocation(FILE_LOCATION); exc.terminate(); }
#else
/// A macro for adding location when throwing an gpstk::Exception
/// @ingroup exceptiongroup
#define GPSTK_THROW(exc)   { exc.addLocation(FILE_LOCATION); throw exc; }
/// A macro for adding location when rethrowing an gpstk::Exception
/// @ingroup exceptiongroup
#define GPSTK_RETHROW(exc) { exc.addLocation(FILE_LOCATION); throw; }
#endif



/**
 * A macro for quickly defining a new exception class that inherits from
 * an gpstk::Exception derived class.  Use this to make specific exceptions,
 * such as the ones defined in this header file.  Make sure that all
 * exceptions have "\@ingroup exceptiongroup" in their comment block
 * so doxygen knows what to do with them.
 *
 * @ingroup exceptiongroup
 */
#define NEW_EXCEPTION_CLASS(child, parent) \
class child : public parent  \
{ \
public: \
      /** Default constructor. */ \
   child() : parent() {} \
      /** Copy constructor. */ \
   child(const child& a): parent(a) {} \
      /** Cast constructor. */ \
   child(const gpstk::Exception& a) : parent(a) {}; \
      /** \
       * Common use constructor. \
       * @param a text description of exception condition. \
       * @param b error code (default none) \
       * @param c severity of exception (default unrecoverable) \
       */ \
   child(const std::string& a, unsigned long b = 0,\
         gpstk::Exception::Severity c = gpstk::Exception::unrecoverable) \
         : parent(a, b, c) \
   {};\
      /** \
       * Common use constructor. \
       * @param a text description of exception condition. \
       * @param b error code (default none) \
       * @param c severity of exception (default unrecoverable) \
       */ \
   child(const char* a, unsigned long b = 0,\
   gpstk::Exception::Severity c = gpstk::Exception::unrecoverable) \
   : parent(a, b, c) \
   {};\
      /** Destructor. */ \
   ~child() {} \
      /** Returns the name of the exception class. */ \
   std::string getName() const {return ( # child);} \
      /** assignment operator for derived exceptions */ \
   child& operator=(const child& kid) \
      { parent::operator=(kid); return *this; } \
      /** ostream operator for derived exceptions */ \
   friend std::ostream& operator<<(std::ostream& s, const child& c) \
      { c.dump(s); return s; } \
}

namespace gpstk
{
      /// Thrown when a function is given a parameter value that it invalid
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(InvalidParameter, Exception);

      /// Thrown if a function can not satisfy a request
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(InvalidRequest, Exception);

      /// Thrown when a required condition in a function is not met.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(AssertionFailure, Exception);

      /// Thrown if a function makes a request of the OS that can't be satisfied.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(AccessError, Exception);

      /// Attempts to access an "array" or other element that doesn't exist
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(IndexOutOfBoundsException, Exception);

      /// A function was passed an invalid argument
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(InvalidArgumentException, Exception);

      /// Application's configuration is invalid
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(ConfigurationException, Exception);

      /// Attempted to open a file that doesn't exist
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(FileMissingException, Exception);

      /// A problem using a system semaphore
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SystemSemaphoreException, Exception);

      /// A problem using a system pipe
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SystemPipeException, Exception);

      /// A problem using a system queue
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SystemQueueException, Exception);

      /// Unable to allocate memory
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(OutOfMemory, Exception);

      /// Operation failed because it was unable to locate the requested obj
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(ObjectNotFound, AccessError);

      /// Attempted to access a null pointer
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(NullPointerException, Exception);

      /// Attempted to access a unimplemented function
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(UnimplementedException, Exception);

} // namespace gpstk
#endif




