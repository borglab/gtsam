#pragma ident "$Id$"

// CommandLine.hpp  Command line options and argument processing.
//
// NB. Repeatable means a parse error will be generated if the option appears more
// than once, unless that option is repeatable. Only vector types (2,5,7) can have
// more than one value, but this has nothing to do with being repeatable!
// NB. Options that go into vectors (types 2,5,7) may have arguments value,value,...
// and @files.lst where files.lst is a file containing values separated by spaces
// and/or linefeed and NOT starting with # (but not val,val,.. inside the file).
// NB. There is an issue for repeatable options that are not vector types, namely if
// there is more than one value, which to use: values[0] or values[values.size()-1]?
// NB. This is related to the question for bool options - does the presence of the
// option set the target true, or does it toggle the target? see TD in .cpp

#ifndef COMMAND_LINE_INCLUDE
#define COMMAND_LINE_INCLUDE

#include "RinexSatID.hpp"
#include <string>
#include <vector>
#include <map>

/// list of Options
class CommandLine {
private:
   /// types of options supported - easily expanded
   typedef enum OptionTypeEnum {
      typeUndefined=-1,
      typeBool,
      typeInt,
      typeVectorInt,
      typeDouble,
      typeString,
      typeVectorString,
      typeSat,
      typeVectorSat,
      typeCount         // this must be last
   } typeOpt;

   // ---------------------------------------------------------------------
   // Encapsulate information needed to use an option, including where to store values
   class Option {
   public:
      char shortOpt;      // character appearing in the short command : x in  -x <arg>
      std::string longOpt;// string appearing in the long command : obs in --obs <arg>
      std::string arg;    // string giving the argument : arg in the above example
      std::string predesc;// string to put on line _before_ description on syntax page
      std::string desc;   // string giving the description on the syntax page
      std::string syntax; // full string used in syntax page: --opt <arg>   Desc (def)
      bool repeat;        // if true, option is repeatable, meaning repeat=F causes a
                          // parse error if the option appears more than once
      bool required;      // if true, option is required
      bool expand;        // if true, expand arguments a,b,c (vector types only)
      typeOpt type;       // type target: bool, int, doub, str, vec<str>,
                          //    RinexSatID, vec<RinexSatID>
      void *p_output;     // pointer to output - default is its value on input

      std::vector<std::string> values;    // store values from command line

      Option(void) : 
         shortOpt(0),repeat(false),required(false),expand(true),
         type(typeUndefined),p_output(NULL) { }

      Option(char s, std::string l, std::string a, std::string predes,
               std::string des, bool rep, bool req, typeOpt t, void *ptr)
      {
         shortOpt = s;
         longOpt = l;
         arg = a;
         predesc = predes;
         desc = des;
         repeat = rep;
         required = req;
         type = t;
         p_output = ptr;
         expand = true;
      }

   }; // end class Option -------------------------------------------------

   // -------------- member data ------------------------------------------
   // first two for internal use only
   bool help,verbose,helponly,foundErrors;
   int debug;        // NB default is -1

   int syntaxPageBuilt;       // 0 c'tor, 1 Usage: ..., 2 prgm descript, 3 complete
   std::string syntaxPage;

   /// list of Options
   std::vector<Option> options;

   /// list of strings '--opt' that, with their following arg, are ignored
   std::vector<std::string> ignore_opts_with_arg;

   /// list of strings '--opt' that are simply ignored
   std::vector<std::string> ignore_opts_without_arg;

   /// list of strings '--opt' that turn "ignoring of args" ON
   std::vector<std::string> ignore_on_opts;

   /// list of strings '--opt' that turn "ignoring of args" OFF
   std::vector<std::string> ignore_off_opts;

   /// for deprecated options - substitutions: if key is found, replace it with value;
   /// value must be the longOpt of an option. Include '--' in the strings
   std::map<std::string,std::string> deprec_opts;

   /// undocumented options : from Add(...,false);
   std::vector<Option> undoc_options;

public:
   // -------------- member functions -------------------------------------
   // constructor
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
    CommandLine(void)
      : help(false),debug(-1),verbose(false),syntaxPageBuilt(0) { }
#pragma clang diagnostic pop
   // destructor
   ~CommandLine(void) { }

   // access
   bool hasHelp(void) { return help; }
   bool hasErrors(void) { return foundErrors; }

   // -------------- add to list ---------------------------------------
   /// add an 'ignore' option, that is an option '--opt' that is simply ignored.
   /// if has_arg is true, the token following --opt (its argument) is also ignored.
   void Add_ignore(std::string opt, bool has_arg=false)
   {
      if(has_arg) ignore_opts_with_arg.push_back(opt);
      else        ignore_opts_without_arg.push_back(opt);
   }

   /// add an 'ignore_on' option; this is an option '--opt' that causes the options
   /// that follow to be ignored, until an 'ignore_off' option is found.
   void Add_ignore_on(std::string opt) { ignore_on_opts.push_back(opt); }

   /// add an 'ignore_off' option; this is an option '--opt' that turns off the
   /// 'ignoring of options' that was started by an 'ignore_on' option.
   void Add_ignore_off(std::string opt) { ignore_off_opts.push_back(opt); }

   /// add a deprecated option, that is let old_opt be equivalent to new_opt,
   /// where new_opt is the long option for one the the regular options.
   void Add_deprecated(std::string old_opt, std::string new_opt) {
      deprec_opts[old_opt] = new_opt;
   }

   /// add a boolean option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      bool* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeBool,(void *)ptr);
      add(opt,doc);
   }

   /// add an integer option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      int* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeInt,(void *)ptr);
      add(opt,doc);
   }

   /// add a vector integer option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      std::vector<int>* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeVectorInt,(void *)ptr);
      add(opt,doc);
   }

   /// add a double option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      double* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeDouble,(void *)ptr);
      add(opt,doc);
   }

   /// add a string option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      std::string* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeString,(void *)ptr);
      add(opt,doc);
   }

   /// add a vector<string> option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      std::vector<std::string>* ptr, std::string predes, std::string des,
      bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeVectorString,(void *)ptr);
      add(opt,doc);
   }

   /// add a RinexSatID option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      gpstk::RinexSatID* ptr, std::string predes, std::string des, bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeSat,(void *)ptr);
      add(opt,doc);
   }

   /// add a vector<RinexSatID> option
   void Add(char s, std::string l, std::string a, bool rep, bool req,
      std::vector<gpstk::RinexSatID>* ptr, std::string predes, std::string des,
      bool doc=true)
   {
      Option opt(s,l,a,predes,des,rep,req,typeVectorSat,(void *)ptr);
      add(opt,doc);
   }

   // -------------------------------------------------------------
   // modify an option - string must match long option string passed to constructor
   // vector types only: turn off expansion of arguments ('a,b,c' -> 'a' 'b' and 'c')
   void noExpansion(std::string l)
   {
      for(size_t i=0; i<options.size(); i++) {
         if(options[i].longOpt == l && (options[i].type == typeVectorString ||
                                        options[i].type == typeVectorSat ||
                                        options[i].type == typeVectorInt))
         {
            options[i].expand = false;
            return;
         }
      }
   }

   // -------------------------------------------------------------
   /// Define the text after 'Usage: '; default is '<prgm> [options] ...'
   void DefineUsageString(std::string str) throw()
   {
      syntaxPage = "Usage: " + str;
      syntaxPageBuilt = 1;
   }

   /// Create the command line = list of commands; parse it
   /// @param argc the number of command line args
   /// @param argv command line args
   /// @param PrgmDesc string giving program description, shown at top of syntax page
   /// @param Usage string return syntax page
   /// @param Errors string return all error messages
   /// @param Unrec vector of strings not recognized by parser
   /// @return -3 an error was found in the definition of command line options
   ///         -2 an error occurred (e.g. new failed)
   ///         -1 an error was found on the command line
   ///          0 ok
   ///          1 help was requested
   int ProcessCommandLine(int argc, char** argv, std::string PrgmDesc,
             std::string& Usage, std::string& Errors, std::vector<std::string>& Unrec)
      throw(gpstk::Exception);

   /// Dump the configuration. Output is of the form
   ///   longOpt Descript : values
   void DumpConfiguration(std::ostream& os) throw(gpstk::Exception);

private:
   /// determine if the command line, as declared, is valid
   bool ValidateCommandLine(std::string& msg) throw(gpstk::Exception);

   /// Build the syntax page
   void BuildSyntaxPage(void) throw(gpstk::Exception);

   /// Preprocess the arguments by pulling out debug, etc, replace deprecated options,
   /// drop ignored options, open --file files, open list files (@file) and parse
   /// comma-separated values
   void PreProcessArgs(const char *arg, std::vector<std::string>& Args,
      std::string& Errors) throw(gpstk::Exception);

   /// Parse the (preprocessed) list of args
   void Parse(std::vector<std::string>& Args, std::string& Errors,
               std::vector<std::string>& Unrecog) throw(gpstk::Exception);

   /// Generate the usage string (syntax page)
   std::string SyntaxPage(void) throw(gpstk::Exception);

   /// Post process - convert value strings to real values
   void Postprocess(std::string& Errors, std::vector<std::string>& Unrecog)
      throw(gpstk::Exception);

   /// Used internally
   void add(Option& opt, bool doc) {
      if(doc)
         options.push_back(opt);
      else
         undoc_options.push_back(opt);
   }

}; // end class CommandLine

#endif // COMMAND_LINE_INCLUDE
