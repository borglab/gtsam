#pragma ident "$Id$"



/**
 * @file CommandOption.hpp
 * Command line options
 */

#ifndef COMMANDOPTION_HPP
#define COMMANDOPTION_HPP

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

#include "getopt.h"

#include <string>
#include <vector>

namespace gpstk
{
      /** @defgroup commandoptiongroup Command-Line Options */
      //@{

      // forward declaration
   class CommandOption;
   typedef std::vector<CommandOption*> CommandOptionVec;

      /// The default command option list used by CommandOption and
      /// gpstk::CommandOptionParser
   extern CommandOptionVec defaultCommandOptionList;

      /**
       * This class is part of a replacement for getopt.
       *
       * Each CommandOption
       * represents an option you would enter at a command line.  You can
       * specify whether the option does or does not require an argument,
       * if the option is of a certain type (string or number), then the
       * short option (i.e. '-f') and long option ('--foo'), whether it's
       * a required option or not, then a short description for a help display.
       * By default, options can appear an unlimited number of times on a
       * command line.  Use setMaxCount() to set a maximum limit to this.
       * In that case, additional appearances of that option will trigger
       * an error once parsed.
       *
       * When a CommandOption is created (not using the default
       * constructor), it automatically adds itself to a list that will
       * be used by gpstk::CommandOptionParser. If you want to manage your
       * own list, pass in your own std::vector<gpstk::CommandOptionParser>
       * with the CommandOption constructor. After parsing the command
       * line, you can use the getValue() and
       * getCount() methods to see what arguments
       * the options had and how many times the option was listed on
       * the command line.
       *
       * This class is strongly connected to gpstk::CommandOptionParser, so
       * if you change anything here, make sure you don't side affect
       * the other.
       *
       * @sa getopttest.cpp in the test directory for some examples.
       *
       * @warning DO NOT USE THE DEFAULT CONSTRUCTOR FOR THIS CLASS.
       * That's for the STL use requirements only.
       * @warning Do not repeat characters or strings used in the short
       * or long command options.  Doing so will cause one of them not to work.
       * @warning Do not reuse the CommandOption objects. Make a separate
       * one for each option you want on the command line.
       * @warning Make the description an understandable, grammatically
       * correct sentence.
       * @warning The resulting behavior of not heeding the above advice
       * is undefined, and I take no responsibility for the results of you
       * not taking appropriate action in light of this warning.
       */
   class CommandOption
   {
   public:
         /// let's the CommandOptionParser see it's private parts =)
      friend class CommandOptionParser;

         /// Every option must either have or not have an argument.
         /// There are no optional arguments because Solaris doesn't support it.
      enum CommandOptionFlag
      {
         noArgument = 0,         ///< option requires no arguments
         hasArgument = 1         ///< option requires an argument
      };

         /// This is so you can limit what type or argument an option can have.
         /// If specified, it will be checked when the argument is encountered.
         /// Errors will get set appropriately if there are any.
      enum CommandOptionType
      {
         trailingType, ///< Special case, no option, just the remaining args.
         stdType,      ///< The argument of this option can be any type.
         metaType      ///< A meta-option that has its own special validation.
      };

         /**
          * Constructor.
          * @param of Whether or not this command requires an argument
          * @param ot The type of option (string, number, any, etc.)
          * @param shOpt The one character command line option.  Set to 0
          * if unused.
          * @param loOpt The long command option.  Set to std::string()
          * if unused.
          * @param desc A string describing what this option does.
          * @param req Set to true if this is a required option.
          * @param optVectorList Use this to create your own
          * command option list if you want to use an alternate method
          * of parsing the command options.
          */
      CommandOption(const CommandOptionFlag of,
                    const CommandOptionType ot,
                    const char shOpt,
                    const std::string& loOpt,
                    const std::string& desc,
                    const bool req = false,
                    CommandOptionVec& optVectorList =
                       defaultCommandOptionList)
            : optFlag(of),  optType(ot),
              shortOpt(shOpt), longOpt(loOpt), description(desc),
              required(req), count(0), maxCount(0), order(0)
         {optVectorList.push_back(this);}

         /// Sets the maximum number of times this should appear on the
         /// command line.
      CommandOption& setMaxCount(const unsigned long l)
         {maxCount = l; return *this;}

         /// Returns a string with the flags for this CommandOption.
         /// (i.e.  "-f | --foo")
      virtual std::string getOptionString() const;

         /// Returns a formatted string with the flags for this CommandOption.
         /// (i.e.  "  -f, --foo=ARG")
      std::string getFullOptionString() const;

         /// Returns a string with the argument format.
      virtual std::string getArgString() const
      { return "ARG"; }

         /// Returns a struct option for use with getopt_long.
      struct option toGetoptLongOption() const;
         /// Returns a string for use with getopt.
      std::string toGetoptShortOption() const;

         /**
          * Returns the number of times this option was found on the
          * command line.
          */
      virtual unsigned long getCount() const { return count; }

         /**
          * Returns the arguments this option had passed in from the
          * command line.
          */
      std::vector<std::string> getValue() const { return value; }

         /// Returns the order which this command option was seen on the
         /// command line, with 1 being the first option.
         //  If it can be repeated, this order represents the order of
         /// the last occurance of this option.
      unsigned int getOrder() const { return order; }

         /// Displays this->value to the stream \c out.
      std::ostream& dumpValue(std::ostream& out) const;

         /// Returns a formatted string with the description of this option.
      std::string getDescription() const;

         /**
          * If you specified a format for the arguments (for example, digit or
          * string), this function checks them to see if they match.
          * If they don't, an error string is returned.  If they do, an
          * empty string is returned.
          * @param optVec complete set of processed command line options
          *   being processed (used by some option types).
          */
      virtual std::string checkArguments();

         /// Destructor
      virtual ~CommandOption() {}

   protected:
         /// Flag for determining whether this option has an argument or not.
      CommandOptionFlag optFlag;
         /// Flag for determining whether this option has a specific
         /// argument type.
      CommandOptionType optType;
         /// The character for the short option (for example, '-f').
      char shortOpt;
         /// The string for the long option (for example, "--foo").
      std::string longOpt;
         /// The description for the help text.
      std::string description;
         /// Any arguments passed with this option get put in here.
      std::vector<std::string> value;
         /// Whether or not this is a required command line option.
      bool required;
         /// The number of times this option was encountered on the
         /// command line.
      unsigned long count;
         /// The maximum number of times this can appear on the command line.
         /// If it's 0, then it's unlimited.
      unsigned long maxCount;
         /// The order in which this option was encountered on the command line
      unsigned long order;

         /// Default Constructor
      CommandOption() {}
   };

      /// A subclass of CommandOption that is a required command line option.
   class RequiredOption : public CommandOption
   {
   public:
         /// Constructor
      RequiredOption(const CommandOptionFlag of,
                     const CommandOptionType ot,
                     const char shOpt,
                     const std::string& loOpt,
                     const std::string& desc)
            : CommandOption(of, ot, shOpt, loOpt, desc, true)
         {}

         /// Destructor
      virtual ~RequiredOption() {}

   protected:
         /// default constructor
      RequiredOption() {}
   };

      /// A subclass of CommandOption that has no arguments
   class CommandOptionNoArg : public CommandOption
   {
   public:
         /// Constructor
      CommandOptionNoArg(const char shOpt,
                         const std::string& loOpt,
                         const std::string& desc,
                         const bool required = false)
            : CommandOption(noArgument, stdType, shOpt, loOpt, desc, required)
         {}

         /// Destructor
      virtual ~CommandOptionNoArg() {}
         /// Returns true if this option was found on the command line
      operator bool() const throw() { return (getCount() != 0); }

   protected:
         /// Default Constructor
      CommandOptionNoArg() {}
   };

      /// A subclass of CommandOption that has an argument.
   class CommandOptionWithArg : public CommandOption
   {
   public:
         /// Constructor
      CommandOptionWithArg(const CommandOptionType ot,
                           const char shOpt,
                           const std::string& loOpt,
                           const std::string& desc,
                           const bool required = false)
            : CommandOption(hasArgument, ot, shOpt, loOpt, desc, required)
         {}

         /// Destructor
      virtual ~CommandOptionWithArg() {}

   protected:
         /// Default Constructor
      CommandOptionWithArg() {}
   };

      /// A CommandOption that requires a string argument.
   class CommandOptionWithAnyArg : public CommandOptionWithArg
   {
   public:
         /// Constructor
      CommandOptionWithAnyArg(const char shOpt,
                              const std::string& loOpt,
                              const std::string& desc,
                              const bool required = false)
            : CommandOptionWithArg(stdType, shOpt, loOpt, desc, required)
         {}

         /// Destructor
      virtual ~CommandOptionWithAnyArg() {}

   protected:
         /// Default Constructor
      CommandOptionWithAnyArg() {}
   };

      /// A CommandOption that requires a string argument.
   class CommandOptionWithStringArg : public CommandOptionWithArg
   {
   public:
         /// Constructor
      CommandOptionWithStringArg(const char shOpt,
                                 const std::string& loOpt,
                                 const std::string& desc,
                                 const bool required = false)
            : CommandOptionWithArg(stdType, shOpt, loOpt, desc, required)
         {}

         /// Destructor
      virtual ~CommandOptionWithStringArg() {}

      virtual std::string checkArguments();

         /// Returns a string with the argument format.
      virtual std::string getArgString() const
      { return "<alpha>"; }

   protected:
         /// Default Constructor
      CommandOptionWithStringArg() {}
   };

      /// A CommandOption that requires a numeric argument.
   class CommandOptionWithNumberArg : public CommandOptionWithArg
   {
   public:
         /// Constructor
      CommandOptionWithNumberArg(const char shOpt,
                                 const std::string& loOpt,
                                 const std::string& desc,
                                 const bool required = false)
            : CommandOptionWithArg(stdType, shOpt, loOpt, desc, required)
         {}

         /// Destructor
      virtual ~CommandOptionWithNumberArg() {}

      virtual std::string checkArguments();

         /// Returns a string with the argument format.
      virtual std::string getArgString() const
      { return "NUM"; }

   protected:
         /// Default Constructor
      CommandOptionWithNumberArg() {}
   };

      /**
       * It only makes sense to have a single one of these set. It is
       * the option that takes the rest of the command line options
       * that are not part of any other options.  e.g. "strace -ofile
       * command arg1 arg2". The "command arg1 arg2" part is placed in
       * objects of this class.
       *
       * @short CommandOption to take the rest of the command line
       */
   class CommandOptionRest : public CommandOptionWithArg
   {
   public:
         /**
          * CommandOptionRest contructor.  This sets the CommandOptionType
          * for this object to trailingType.
          *
          * @param desc short description of the option
          * @param required true if option is required
          */
      CommandOptionRest(const std::string& desc,
                        const bool required = false)
            : CommandOptionWithArg(trailingType, 0, "", desc, required)
      {}

         /// Destructor
      virtual ~CommandOptionRest() {}

      virtual std::string checkArguments();

   protected:
         /// Default Constructor
      CommandOptionRest() {}
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to make sure at least one of a set of real
       * options has been used.
       * \warning There's nothing to prevent you from, say, adding
       * another meta-option to the list of mutually exclusive options
       * contained in a CommandOptionOneOf instance (or even itself),
       * but the behavior if you try this is undefined.
       */
   class CommandOptionOneOf : public CommandOption
   {
   public:
         /**
          * CommandOptionOneOf contructor.  This sets the CommandOptionType
          * for this object to metaType.
          */
      CommandOptionOneOf()
            : CommandOption(noArgument, metaType, 0, "", "")
      {}

         /// Destructor
      virtual ~CommandOptionOneOf() {}

      virtual std::string checkArguments();

         /// Add an option to the list of mutually exclusive options
      void addOption(CommandOption* opt)
      { optionVec.push_back(opt); }

         /// @return the command option that was used (NULL if none).
      CommandOption* whichOne() const;

   protected:
      CommandOptionVec optionVec;
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to make sure that if one of a set of real options
       * has been used, all of the set are used.
       * \warning There's nothing to prevent you from, say, adding
       * another meta-option to the list of mutually exclusive options
       * contained in a CommandOptionAllOf instance (or even itself),
       * but the behavior if you try this is undefined.
       */
   class CommandOptionAllOf : public CommandOptionOneOf
   {
   public:
         /**
          * CommandOptionAllOf contructor.  This sets the CommandOptionType
          * for this object to metaType.
          */
      CommandOptionAllOf()
      {}

         /// Destructor
      virtual ~CommandOptionAllOf() {}

      virtual std::string checkArguments();

         /// returns the sum of all encapsulated option counts if all are in use, zero otherwise.
      virtual unsigned long getCount() const;

   private:
         // hide this as it doesn't make sense for this class
      CommandOption* whichOne() const;
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to enforce mutual exclusion between a set of real
       * options.
       * \warning There's nothing to prevent you from, say, adding
       * another mutex to the list of mutually exclusive options
       * contained in a CommandOptionMutex instance (or even itself),
       * but the behavior if you try this is undefined.
       */
   class CommandOptionMutex : public CommandOptionOneOf
   {
   public:
         /**
          * CommandOptionMutex contructor.  This sets the CommandOptionType
          * for this object to metaType.
          *
          * @param required true if option is required.  This makes
          * CommandOptionMutex do CommandOptionOneOf-type checking in
          * addition to the exclusion (i.e. it checks to make sure at
          * least one option was specified).
          */
      CommandOptionMutex(const bool required = false)
            : doOneOfChecking(required)
      {}

         /// Destructor
      virtual ~CommandOptionMutex() {}

      virtual std::string checkArguments();

   protected:
      bool doOneOfChecking;
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to make sure that a required option is set where
       * the requirement is based on another option (that is, if you
       * specify one, you must have specified another).
       * \warning There's nothing to prevent you from using other meta
       * options as requirements, but the behavior if you try this is
       * undefined.
       */
   class CommandOptionDependent : public CommandOption
   {
   public:
         /**
          * CommandOptionDependent contructor.  This sets the
          * CommandOptionType for this object to metaType.  During
          * command line option validation, if \c child is set, \c
          * parent is checked to make sure it is also set.
          *
          * @param parent Command option that must be used if...
          * @param child ...is used.
          */
      CommandOptionDependent(const CommandOption* parent,
                             const CommandOption* child)
            : CommandOption(noArgument, metaType, 0, "", ""),
              requiree(parent), requirer(child)
      {}

         /// Destructor
      virtual ~CommandOptionDependent() {}

      virtual std::string checkArguments();

   protected:
         /// Default Constructor
      CommandOptionDependent() {}

      const CommandOption *requiree, *requirer;
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to group other options together for use in other
       * meta-options.  This particular meta-option allows a group of
       * options to be specified in other meta-options.  This option
       * is considered "set" if any of the member options are set.
       * This class and CommandOptionGroupAnd are designed to make up
       * for the fact that the verification meta-options are not
       * designed to work in other verification meta-options.
       */
   class CommandOptionGroupOr : public CommandOptionOneOf
   {
   public:
         /**
          * CommandOptionGroupOr contructor.  Does nothing explicitly.
          */
      CommandOptionGroupOr()
      {}

         /// Destructor.
      virtual ~CommandOptionGroupOr() {}

         /// Do not do any checking.
      virtual std::string checkArguments() { return std::string(); }

         /// returns the sum of all encapsulated option counts.
      virtual unsigned long getCount() const;

         /// return a string containing the aggregated option strings
      virtual std::string getOptionString() const;
   };

      /**
       * This is a special "command option" which is really a
       * meta-option to group other options together for use in other
       * meta-options.  This particular meta-option allows a group of
       * options to be specified in other meta-options.  This option
       * is considered "set" iff all of the member options are set.
       * This class and CommandOptionGroupAnd are designed to make up
       * for the fact that the verification meta-options are not
       * designed to work in other verification meta-options.
       */
   class CommandOptionGroupAnd : public CommandOptionGroupOr
   {
   public:
         /**
          * CommandOptionGroupAnd contructor.  Does nothing explicitly.
          */
      CommandOptionGroupAnd()
      {}

         /// Destructor.
      virtual ~CommandOptionGroupAnd() {}

         /// returns the sum of all encapsulated option counts if all are in use, zero otherwise.
      virtual unsigned long getCount() const;
   };

      //@}

} // namespace gpstk
#endif
