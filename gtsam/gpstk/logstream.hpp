/// @file logstream.hpp
/// Writing to a log stream made easy, typesafe, threadsafe and portable.
/// Inspired by Petru Marginean, "Logging in C++," Dr.Dobbs, October 2007.

#ifndef LOGSTREAMINCLUDE
#define LOGSTREAMINCLUDE

#include <cstdio>
#include <sstream>
#include <string>
#include <iostream>

/// levels that the user may give the log stream output in the output statement,
/// e.g. LOG(ERROR) << "This is an error message"; DEBUGn levels appear indented
/// by 2*n spaces in the log stream. Default level is INFO.
enum LogLevel {ERROR, WARNING, INFO, VERBOSE, DEBUG,
               DEBUG1, DEBUG2, DEBUG3, DEBUG4, DEBUG5, DEBUG6, DEBUG7 };
/// Define the maximum log level
#ifndef FILELOG_MAX_LEVEL
#define FILELOG_MAX_LEVEL DEBUG7
#endif

/// template class Log is used by classes ConfigureLOG and ConfigureLOGstream.
/// class ConfigureLOGstream is just class Log with template type = ConfigureLOGstream
template <class T> class Log
{
public:
   Log() {};
   virtual ~Log();
   /// write out to log stream at level, default is INFO
   std::ostringstream& Put(LogLevel level = INFO);

   /// get/set; if true, output time tag on each line, first
   static bool& ReportTimeTags();
   /// get/set; if true, output level on each line, after the time tag
   static bool& ReportLevels();
   /// get/set the highest level that is actually output to log stream
   static LogLevel& ReportingLevel();
   /// convert a level to a string corresponding to the LogLevel enum
   static std::string ToString(LogLevel level);
   /// convert from a string (= names in the LogLevel enum) to a LogLevel
   static LogLevel FromString(const std::string& level);

protected:
   /// string stream to which output is written; destructor will dump to log stream.
   std::ostringstream os;

#ifdef WIN32                  // see kludge note below
   static LogLevel reportingLevel;  ///< static data for ReportingLevel()
   static bool dumpTimeTags;        ///< static data for ReportTimeTags()
   static bool dumpLevels;          ///< static data for ReportLevels()
#endif

private:
   Log(const Log&);                 ///< do not implement
   Log& operator=(const Log&);      ///< do not implement
   std::string NowTime(void);       ///< generate a timetag as string
};

template <class T> std::ostringstream& Log<T>::Put(LogLevel level)
{
   if(Log<T>::ReportTimeTags()) os << NowTime() << " ";
   if(Log<T>::ReportLevels()) {
      os << ToString(level) << ": ";
      // add indentation for deep debug levels
      if(level > DEBUG) os << std::string(2*(level-DEBUG),' ');
   }
   return os;
}

template <class T> Log<T>::~Log()
{
   os << std::endl;           // TD make optional?
   T::Output(os.str());
}

template <class T> bool& Log<T>::ReportLevels()
{
#ifndef WIN32                 // see kludge note below
   static bool dumpLevels = false;
#endif
   return dumpLevels;
}

template <class T> bool& Log<T>::ReportTimeTags()
{
#ifndef WIN32                 // see kludge note below
   static bool dumpTimeTags = false;
#endif
   return dumpTimeTags;
}

template <class T> LogLevel& Log<T>::ReportingLevel()
{
#ifndef WIN32                 // see kludge note below
   static LogLevel reportingLevel = INFO; // FILELOG_MAX_LEVEL;
#endif
   return reportingLevel;
}

template <class T> std::string Log<T>::ToString(LogLevel level)
{
   // note enum LogLevel above
	static const char* const buffer[] = {"ERROR", "WARNING", "INFO", "VERBOSE",
      "DEBUG","DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4", "DEBUG5", "DEBUG6", "DEBUG7" };
   return buffer[level];
}

template <class T> LogLevel Log<T>::FromString(const std::string& level)
{
   // note enum LogLevel above
   if(level == "DEBUG7")  return DEBUG7;
   if(level == "DEBUG6")  return DEBUG6;
   if(level == "DEBUG5")  return DEBUG5;
   if(level == "DEBUG4")  return DEBUG4;
   if(level == "DEBUG3")  return DEBUG3;
   if(level == "DEBUG2")  return DEBUG2;
   if(level == "DEBUG1")  return DEBUG1;
   if(level == "DEBUG")   return DEBUG;
   if(level == "VERBOSE") return VERBOSE;
   if(level == "INFO")    return INFO;
   if(level == "WARNING") return WARNING;
   if(level == "ERROR")   return ERROR;
   Log<T>().Put(WARNING)
       << "Unknown logging level '" << level << "'. Using INFO level instead.";
   return INFO;
}

// time tag - platform dependent -----------------------------------------
//#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#ifdef WIN32
#include <sys/timeb.h>
template <class T> inline std::string Log<T>::NowTime()
{
   _timeb t;
   _ftime(&t);
   long sec=t.time;
   double dt=double(sec)+t.millitm/1000.;
   sec -= long(dt/86400)*86400;
   dt = double(sec)+t.millitm/1000.;
   char result[100] = {0};
   int h=int(dt/3600.);
   dt -= h*3600.;
   int m=int(dt/60.);
   dt -= m*60.;
   int s=int(dt);
   dt -= s;
   std::sprintf(result,"%02d:%02d:%02d.%03d",h,m,s,int(dt*1000.));
   return result;
}

#else    // not WIN32

#include <sys/time.h>
template <class T> inline std::string Log<T>::NowTime()
{
   char buffer[11];
   time_t t;
   time(&t);
   tm r = {0};
   strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
   struct timeval tv;
   gettimeofday(&tv, 0);
   char result[100] = {0};
   std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000); 
   return result;
}

#endif //WIN32

// ------- end class LOG

/// class ConfigureLOGstream
/// Configure and write to a log stream; type-safe, thread-safe and very portable.
/// Inspired by Petru Marginean, "Logging in C++," Dr.Dobbs, October 2007.
///
/// How to use: 1. include logstream.hpp in all modules that will write to the log,
///    and (optional) define the stream. If stream is not defined, std::cout is used.
/// @code
///    #include "logstream.hpp"
///    #include <fstream>
///    //...
///    std::ofstream oflog("test.log",std::ios_base::out);
///    ConfigureLOG::Stream() = &oflog; // else use &(std::cout) - the default
/// @endcode
///
/// How to use: 2. set switches to output time tags and level (default = false)
/// @code
///    ConfigureLOG::ReportLevels() = true;
///    ConfigureLOG::ReportTimeTags() = true;
/// @endcode
///
/// How to use: 3. set the maximum level to output. Choices (in enum LogLevel) are:
/// ERROR,WARNING,INFO,VERBOSE,DEBUG,DEBUG1,DEBUG2,DEBUG3,DEBUG4,DEBUG5,DEBUG6,DEBUG7
/// @code
///    ConfigureLOG::ReportingLevel() = ConfigureLOG::Level("DEBUG");
///    // (default is INFO, so ERROR, WARNING, and INFO are reported)
/// @endcode
///
/// NB. All the ConfigureLOG:: settings are defined in ALL linked modules that
/// includes logstream.hpp, whenever they are set in ANY such module.
/// 
/// How to use: 4. write to LOG(level) where level is one of the LogLevel choices.
///                Only output at or below the ReportingLevel() will appear in log.
/// @code
///    LOG(INFO) << "In main(), level is " << ConfigureLOG::ReportingLevel();
///    LOG(INFO) << "This is an INFO message in main";
///    LOG(WARNING) << "This is a WARNING message in main";
///    LOG(ERROR) << "This is an ERROR message in main";
///    // the output looks like this (time is UT):
///    //19:48:02.691 INFO: In main(), level is 4
///    //19:48:02.691 INFO: This is an INFO message in main
///    //19:48:02.691 WARNING: This is a WARNING message in main
///    //19:48:02.691 ERROR: This is an ERROR message in main
/// @endcode
///
/// How to use: 5. Change ReportingLevel() or the output Stream() at any time.
/// @code
///    std::ofstream ofs("test.new.log",std::ios_base::out);
///    ConfigureLOG::Stream() = &ofs;       // change the log stream
///    ConfigureLOG::ReportingLevel() = ConfigureLOG::Level("DEBUG7");
///                                         // optionally, also change the max level
///    LOG(VERBOSE) << "This is a second, special log file");
///    LOG(DEBUG6) << " ... messages ...";
///    //...
///    ofs.close();                         // optional - not necessary for LOG
///    ConfigureLOG::Stream() = &oflog;     // change back to the original log
///    ConfigureLOG::ReportingLevel() = ConfigureLOG::Level("VERBOSE");
///                                         // optionally, also change the max level
///    // ...
/// @endcode
///
class ConfigureLOGstream
{
public:
   /// direct log stream output to an already opened stream, for example:
   /// @code
   ///    std::ofstream oflog("mylog.log",std::ios_base::out);
   ///    ConfigureLOGstream::Stream() = oflog;
   /// @endcode
   static std::ostream*& Stream();

   /// used internally
   static void Output(const std::string& msg);
};

inline std::ostream*& ConfigureLOGstream::Stream()
{
   static std::ostream *pStream = &(std::cout);
   return pStream;
}

inline void ConfigureLOGstream::Output(const std::string& msg)
{   
   std::ostream *pStream = Stream();
   if(!pStream) return;
   *pStream << msg << std::flush;
}

//----- end class ConfigureLOGstream

/// class ConfigureLOG - inherits class Log with type ConfigureLOGstream
class ConfigureLOG : public Log<ConfigureLOGstream> {
public:
   static std::ostream*& Stream()
   { return ConfigureLOGstream::Stream(); }
   static LogLevel Level(const std::string& str)
   { return FromString(str); }
};

//----- end class ConfigureLOG

// This is a terrible kludge that nevertheless is necessary under Windows 2003 and
// 2005 because global optimization (compiler switch /O1 or /O2 or /Og) causes the
// reportingLevel to be defined ONLY in the module in which ReportingLevel() is
// assigned. Rather than give up optimization or ReportingLevel(), I came up with
// this kludge, which is to remove reportingLevel from the ReportingLevel() function
// and make it a static member of the class, then initialize it outside the class.
// Similarly for dumpTimeTags and dumpLevels.
// Notice that this ought to fail (!) because putting this initialization in the
// include file means that it occurs more than once - in every module in which it
// appears. However Windows allows this; hence I have commented out the inner macro
// here. Ah, the joys of developing under MS....
#ifdef WIN32
//#ifndef LOGSTREAM_INITIALIZE_REPORTING_LEVEL
//#define LOGSTREAM_INITIALIZE_REPORTING_LEVEL
template<> LogLevel Log<ConfigureLOGstream>::reportingLevel = INFO;
template<> bool Log<ConfigureLOGstream>::dumpTimeTags = false;
template<> bool Log<ConfigureLOGstream>::dumpLevels = false;
//#endif
#endif

/// define the macro that is used to write to the log stream
#define LOG(level) \
   if(level <= FILELOG_MAX_LEVEL && \
      level <= ConfigureLOG::ReportingLevel() && \
      ConfigureLOGstream::Stream()) ConfigureLOG().Put(level)

// conveniences
#define pLOGstrm ConfigureLOGstream::Stream()
#define LOGstrm *(ConfigureLOGstream::Stream())
#define LOGlevel ConfigureLOG::ReportingLevel()
//#define showLOGlevel ConfigureLOG::ReportLevels()
//#define showLOGtime ConfigureLOG::ReportTimeTags()

/// NB. Under Windows 2003 and 2005, when this file is included in a module other
/// than the one in which ReportingLevel() is assigned, the assignment does not apply
/// unless global optimization is turned off. Use kludge (above) rather than do this.
//#pragma optimize("g",off)      // turn off global optimization
//#pragma optimize("",on)        // turn on optimizations turned on by compiler switch

#endif //LOGSTREAMINCLUDE --- end of the include file. Test code follows in comments.

/* Example:
// testls.cpp
#include "logstream.hpp"
#include <fstream>

void func(void);                   // testls0.cpp

int main(int argc, char **argv)
{
   try {
      std::ofstream oflog("testls.log",std::ios_base::out);
      ConfigureLOG::Stream() = &oflog;
      ConfigureLOG::ReportLevels() = true;
      ConfigureLOG::ReportTimeTags() = true;
      ConfigureLOG::ReportingLevel() = ConfigureLOG::Level("VERBOSE");

      LOG(INFO) << "In main(), level is "
         << ConfigureLOG::ToString(ConfigureLOG::ReportingLevel());
      LOG(INFO) << "This is an INFO message in main";
      LOG(WARNING) << "This is a WARNING message in main";
      LOG(ERROR) << "This is an ERROR message in main";

      func();
      LOG(INFO) << "In main(), level is "
         << ConfigureLOG::ToString(ConfigureLOG::ReportingLevel());

      LOG(INFO) << "Now write to testls.new.log";
      std::ofstream oflognew("testls.new.log",std::ios_base::out);
      ConfigureLOG::Stream() = &oflognew;
      ConfigureLOG::ReportTimeTags() = false;
      LOG(INFO) << "Change the level to DEBUG7 and turn off time tags";
      ConfigureLOG::ReportingLevel() = ConfigureLOG::FromString("DEBUG7");

      LOG(INFO) << "This is file testls.new.log";
      LOG(INFO) << "In main(), level is now "
         << ConfigureLOG::ToString(ConfigureLOG::ReportingLevel());
      LOG(DEBUG) << "DEBUG message";
      LOG(DEBUG1) << "DEBUG1 message";
      LOG(DEBUG2) << "DEBUG2 message";
      LOG(DEBUG3) << "DEBUG3 message";
      LOG(DEBUG4) << "DEBUG4 message";
      LOG(DEBUG5) << "DEBUG5 message";
      LOG(DEBUG6) << "DEBUG6 message";
      LOG(DEBUG7) << "DEBUG7 message";

      LOG(INFO) << "Now go back to the original log file";
      ConfigureLOG::Stream() = &oflog;

      LOG(INFO) << "This is file testls.log again";
      LOG(INFO) << "In main(), level is now "
         << ConfigureLOG::ToString(ConfigureLOG::ReportingLevel());

      LOG(INFO) << "Change the level to DEBUG3 and turn on time tags";
      ConfigureLOG::ReportTimeTags() = true;
      ConfigureLOG::ReportingLevel() = ConfigureLOG::FromString("DEBUG3");
      LOG(DEBUG) << "DEBUG message";
      LOG(DEBUG1) << "DEBUG1 message";
      LOG(DEBUG2) << "DEBUG2 message";
      LOG(DEBUG3) << "DEBUG3 message";
      LOG(DEBUG4) << "DEBUG4 message";
      LOG(DEBUG5) << "DEBUG5 message";
      LOG(DEBUG6) << "DEBUG6 message";
      LOG(DEBUG7) << "DEBUG7 message";

      return 0;
   }
   catch(std::exception& e) {
      std::cerr << "Exception: " << e.what() << std::endl;
   }
   catch(...) {
      std::cerr << "Unknown exception" << std::endl;
   }
   return 1;
}
// testls0.cpp
#include "logstream.hpp"

void func(void)
{
   LOG(INFO) << "Here in func(), level is "
         << ConfigureLOG::ToString(ConfigureLOG::ReportingLevel());
   LOG(DEBUG) << "This is a debug statement inside func()";

   LOG(INFO) << "Change the level to DEBUG7";
   ConfigureLOG::ReportingLevel() = ConfigureLOG::FromString("DEBUG7");
   LOG(DEBUG5) << "DEBUG5 message";
   LOG(DEBUG6) << "DEBUG6 message";
   LOG(DEBUG7) << "DEBUG7 message";
   LOG(INFO) << "Change the level to DEBUG";
   ConfigureLOG::ReportingLevel() = ConfigureLOG::FromString("DEBUG");
   LOG(INFO) << "Leaving func()";
}
// execution:
C:\>testls

C:\>cat testls.log
20:56:30.353 INFO: In main(), level is VERBOSE
20:56:30.353 INFO: This is an INFO message in main
20:56:30.353 WARNING: This is a WARNING message in main
20:56:30.353 ERROR: This is an ERROR message in main
20:56:30.353 INFO: Here in func(), level is VERBOSE
20:56:30.353 INFO: Change the level to DEBUG7
20:56:30.353 DEBUG5:           DEBUG5 message
20:56:30.353 DEBUG6:             DEBUG6 message
20:56:30.353 DEBUG7:               DEBUG7 message
20:56:30.353 INFO: Change the level to DEBUG
20:56:30.353 INFO: Leaving func()
20:56:30.353 INFO: In main(), level is DEBUG
20:56:30.353 INFO: Now write to testls.new.log
INFO: This is file testls.log again
INFO: In main(), level is now DEBUG7
INFO: Change the level to DEBUG3 and turn on time tags
20:56:30.353 DEBUG: DEBUG message
20:56:30.353 DEBUG1:   DEBUG1 message
20:56:30.353 DEBUG2:     DEBUG2 message
20:56:30.353 DEBUG3:       DEBUG3 message

C:\>cat testls.new.log
INFO: Change the level to DEBUG7 and turn off time tags
INFO: This is file testls.new.log
INFO: In main(), level is now DEBUG7
DEBUG: DEBUG message
DEBUG1:   DEBUG1 message
DEBUG2:     DEBUG2 message
DEBUG3:       DEBUG3 message
DEBUG4:         DEBUG4 message
DEBUG5:           DEBUG5 message
DEBUG6:             DEBUG6 message
DEBUG7:               DEBUG7 message
INFO: Now go back to the original log file
*/
