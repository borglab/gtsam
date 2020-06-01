#pragma ident "$Id$"

/**
 * @file ConfDataReader.cpp
 * Class to parse and manage configuration data files.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008
//
//============================================================================




#include <gtsam/configReader/ConfDataReader.hpp>

using namespace std;

namespace gpstk
{

      // Method to store conf data in this class' data map
   void ConfDataReader::loadData(void)
      throw(ConfigurationException)
   {

         // By default, section name is "DEFAULT"
      std::string sectionName("DEFAULT");

         // Do this until end-of-file reached or something else happens
      while(1)
      {
         try
         {

            std::string line;
            std::string variable;
            std::string value;

            formattedGetLine(line, true);

               // If line is too long, we throw an exception
            if (line.size()>255)
            {
               ConfigurationException e(
                                    "Line too long in configuration file '" +
                                    filename + "'." );
               GPSTK_THROW(e);
            }
              // Skip the blank line
            if(line.size()<1) continue;

               // Let's find and strip comment lines
            if( (StringUtils::firstWord(line)[0] == '#') ||
                (StringUtils::firstWord(line)[0] == ';')  )
            {
               formattedGetLine(line, true);
            }

               // Let's strip comments at the end of lines
            std::string::size_type idx(line.find('#'));
            if( !(idx == std::string::npos) )
            {
               line = line.substr(0, idx);
            }

            idx = line.find(';');
            if( !(idx == std::string::npos) )
            {
               line = line.substr(0, idx);
            }

               // Remove trailing and leading blanks
            line = StringUtils::strip(line);

               // Skip blank lines
            if (line.size()==0)
            {
               continue;
            }


               // Let's start to get data out of file

               // First, handle section names

               // Test if this line declares a new section. Check for '['
            idx = line.find('[');
            if( !(idx == std::string::npos) )
            {

                  // Now, check if there is a closing ']'
               std::string::size_type idx2(line.find(']'));
               if( !(idx2 == std::string::npos) )
               {
                     // Extract name and remove trailing and leading blanks
                  line = StringUtils::strip( line.substr(idx+1, idx2-idx-1) );

                     // Check if section name is appropriate
                  if( checkName(line) )
                  {

                        // Update 'sectionName': make it uppercase
                     sectionName = StringUtils::upperCase(line);

                  }
                  else
                  {
                        // Throw an exception if section name isn't appropriate
                     ConfigurationException e(
                                          "Section name '" +
                                          line + "' in configuration file '" +
                                          filename +
                                          "' does not comply with rules.");

                     GPSTK_THROW(e);
                  }

                     // If this was a section line, continue with next line
                  continue;

               }
               else
               {
                     // Throw an exception if section line is not closed
                  ConfigurationException e(
                                       "Section line '" +
                                       line +
                                       "' in configuration file '" +
                                       filename +
                                       "' was improperly closed" );

                  GPSTK_THROW(e);
               }

            }

               // Second, handle variables

               // Separate variable name from value. Look for separators
            idx = line.find('=');
            if( idx == std::string::npos )
            {
               idx = line.find(':');
            }


               // If we found a separator, keep processing
            if( !(idx == std::string::npos) )
            {

                  // Read variable and value
               variable = StringUtils::strip( line.substr(0, idx) );
               value = StringUtils::strip( line.substr(idx+1) );

                  // Now separate comments

                  // Work on 'variable'
               std::string varComment;

               idx = variable.find(',');
               if( !(idx == std::string::npos) )
               {
                  varComment = StringUtils::strip(variable.substr(idx+1));
                  variable   = StringUtils::strip(variable.substr(0, idx));
               }

                  // Check if variable name is appropriate
               if( checkName(variable) )
               {
                     // Make 'variable' uppercase
                  variable = StringUtils::upperCase(variable);

               }
               else
               {
                     // Throw an exception if variable name isn't appropriate
                  ConfigurationException e(
                                       "Variable name '" +
                                       variable + "' in configuration file '" +
                                       filename +
                                       "' does not comply with rules.");

                  GPSTK_THROW(e);
               }

                  // Now work on 'value'
               std::string valueComment;

               idx = value.find(',');
               if( !(idx == std::string::npos) )
               {
                  valueComment = StringUtils::strip(value.substr(idx+1));
                  value        = StringUtils::strip(value.substr(0, idx));
               }

                  // Store configuration data
               variableData varData;
               varData.varComment   = varComment;
               varData.value        = value;
               varData.valueComment = valueComment;

               confData[sectionName][variable] = varData;

            }

         }  // End of try block
         catch (ConfigurationException& e)
         {
            GPSTK_THROW(e);
         }
         catch (EndOfFile& e)
         {

               // Initialize itCurrentSection
            itCurrentSection = confData.begin();

            return;

         }
         catch (...)
         {

            return;

         }

      } // End of 'while(1)'

   }  // End of method 'ConfDataReader::loadData()'



      /* Method to get the value of a given variable as a string
       *
       * @param variable   Variable name.
       * @param section    Section the variable belongs to.
       *
       */
   string ConfDataReader::getValue( string variable,
                                    string section,
                                    string defaultVal )
      throw(ConfigurationException)
   {

         // Let's make sure that section and variable names are uppercase
      section  = StringUtils::upperCase(section);
      variable = StringUtils::upperCase(variable);


      try
      {

            // Auxiliar variable to store current 'issueException' state
         bool exceptionState( getIssueException() );

            // If 'fallback2Default' is set, and this is NOT the 'DEFAULT'
            // section, we need to temporarily disable 'issueException'.
            // This implies that there is no fallback for 'DEFAULT' variables
         if( (section != "DEFAULT") && (section != "") )
         {
            if( getFallback2Default() )
            {
               setIssueException(false);
            }
         }


            // Check if section and variable exist
         if( ifExist(variable, section) )
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

            return confData[section][variable].value;

         }
         else
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

               // If 'fallback2Default' is set, check also in 'DEFAULT' section
            if ( getFallback2Default() )
            {

               if( ifExist(variable) )
               {

                  return confData["DEFAULT"][variable].value;

               }
               else
               {

                  return defaultVal;

               }

            }
            else
            {

               return defaultVal;

            }  // End of 'if ( getFallback2Default() )'

         }  // End of 'if( ifExist(variable, section) )'

      }
      catch (ConfigurationException& e)
      {

         GPSTK_RETHROW(e);

      }  // End of 'try-catch' block

   }  // End of method 'ConfDataReader::getValue()'



      /* Method to get the value of a given variable as a boolean
       *
       * @param variable   Variable name.
       * @param section    Section the variable belongs to.
       *
       */
   bool ConfDataReader::getValueAsBoolean( string variable,
                                           string section,
                                           bool   defaultVal )
      throw(ConfigurationException)
   {

         // Let's make sure that section and variable names are uppercase
      section  = StringUtils::upperCase(section);
      variable = StringUtils::upperCase(variable);

      try
      {

            // Declare result variable
         string result( getValue( variable, section ) );


            // Test if result is empty (variable does not exist)
         if( result == "" )
         {
               // Return false if variable is empty. Be aware that an empty
               // variable is NOT the same as an unexistent variable
            return defaultVal;

         }


            // 'result' isn't empty. Convert it to uppercase
         result = StringUtils::upperCase(result);

            // Test if it is "TRUE" or "FALSE"
         if( result == "TRUE" )
         {

            return true;

         }
         else
         {

            if( result == "FALSE" )
            {

               return false;

            }
            else
            {

                  // Throw an exception if value is neither TRUE nor FALSE
               ConfigurationException e(
                                    "Variable name '" +
                                    variable + "' in configuration file '" +
                                    filename +
                                    "' is neither TRUE nor FALSE.");

               GPSTK_THROW(e);

            }  // End of 'if( result == "FALSE" )'

         }  // End of 'if( result == "TRUE" )'

      }
      catch (ConfigurationException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'ConfDataReader::getValueAsBoolean()'



      /* Method to fetch (as string) the first value of a given
       * variable list.
       *
       * In this context, a variable list is the same as a variable but
       * it is composed of several parts (words), separated by spaces.
       *
       * @param variableList   Variable list name.
       * @param section        Section the variable list belongs to.
       *
       * \warning This method will MODIFY the original content of
       * 'variableList'.
       */
   string ConfDataReader::fetchListValue( string variableList,
                                          string section,
                                          string defaultVal )
      throw(ConfigurationException)
   {

      try
      {

            // Let's make sure that section and variable names are uppercase
         section      = StringUtils::upperCase(section);
         variableList = StringUtils::upperCase(variableList);

            // Store the original value of 'variableList'
         string origValue( getValue(variableList, section) );

            // Get the first word in 'originalValue'
         string firstValue( StringUtils::stripFirstWord(origValue) );

            // Modify the originalValue value
         confData[section][variableList].value = StringUtils::strip(origValue);

            // Return the first value
         string value = StringUtils::strip(firstValue);
         return ( (value=="") ? string(defaultVal) : value );

      }
      catch (ConfigurationException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'ConfDataReader::fetchListValue()'



      /* Method to fetch (as boolean) the first value of a given
       * variable list.
       *
       * In this context, a variable list is the same as a variable but
       * it is composed of several parts (words), separated by spaces.
       *
       * @param variableList   Variable list name.
       * @param section        Section the variable list belongs to.
       *
       * \warning This method will MODIFY the original content of
       * 'variableList'.
       */
   bool ConfDataReader::fetchListValueAsBoolean( string variableList,
                                                 string section,
                                                 bool   defaultVal )
      throw(ConfigurationException)
   {

      try
      {

            // Let's make sure that section and variable names are uppercase
         section      = StringUtils::upperCase(section);
         variableList = StringUtils::upperCase(variableList);

            // Get first value in 'variableList' and make it uppercase
         string result( fetchListValue(variableList,section) );

         result = StringUtils::upperCase(result);

            // Test if it is "TRUE" or "FALSE"
         if( result == "TRUE" )
         {

            return true;

         }
         else
         {

            if( (result == "FALSE") ||
                (result == "") )    // If list is empty returns false
            {

               return defaultVal;

            }
            else
            {

                  // Throw an exception if value is neither TRUE nor FALSE
               ConfigurationException e(
                                    "Variable list '" +
                                    variableList + "' in configuration file '" +
                                    filename +
                                    "' have a value that is neither TRUE " +
                                    "nor FALSE.");

               GPSTK_THROW(e);

            }  // End of 'if( result == "FALSE" )'

         }  // End of 'if( result == "TRUE" )'

      }  // End of 'try' block
      catch (ConfigurationException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'ConfDataReader::fetchListValueAsBoolean()'



      /* Method to get the description of a given variable
       *
       * @param variable   Variable name.
       * @param section    Section the variable belongs to.
       *
       */
   string ConfDataReader::getVariableDescription( string variable,
                                                  string section )
      throw(ConfigurationException)
   {

         // Let's make sure that section and variable names are uppercase
      section  = StringUtils::upperCase(section);
      variable = StringUtils::upperCase(variable);

      try
      {

            // Auxiliar variable to store current 'issueException' state
         bool exceptionState( getIssueException() );

            // If 'fallback2Default' is set, and this is NOT the 'DEFAULT'
            // section, we need to temporarily disable 'issueException'.
            // This implies that there is no fallback for 'DEFAULT' variables
         if( (section != "DEFAULT") && (section != "") )
         {
            if( getFallback2Default() )
            {
               setIssueException(false);
            }
         }


            // Check if section and variable exist
         if( ifExist(variable, section) )
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

            return confData[section][variable].varComment;

         }
         else
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

               // If 'fallback2Default' is set, check also in 'DEFAULT' section
            if ( getFallback2Default() )
            {

               if( ifExist(variable) )
               {

                  return confData["DEFAULT"][variable].varComment;

               }
               else
               {

                  return "";

               }

            }
            else
            {

               return "";

            }  // End of 'if ( getFallback2Default() )'

         }  // End of 'if( ifExist(variable, section) )'

      }
      catch (ConfigurationException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'ConfDataReader::getVariableDescription()'



      /* Method to get the description of a given value
       *
       * @param variable   Variable name.
       * @param section    Section the variable belongs to.
       *
       */
   string ConfDataReader::getValueDescription( string variable,
                                               string section )
      throw(ConfigurationException)
   {

         // Let's make sure that section and variable names are uppercase
      section  = StringUtils::upperCase(section);
      variable = StringUtils::upperCase(variable);

      try
      {

            // Auxiliar variable to store current 'issueException' state
         bool exceptionState( getIssueException() );

            // If 'fallback2Default' is set, and this is NOT the 'DEFAULT'
            // section, we need to temporarily disable 'issueException'.
            // This implies that there is no fallback for 'DEFAULT' variables
         if( (section != "DEFAULT") && (section != "") )
         {
            if( getFallback2Default() )
            {
               setIssueException(false);
            }
         }


            // Check if section and variable exist
         if( ifExist(variable, section) )
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

            return confData[section][variable].valueComment;

         }
         else
         {

               // Reset 'issueException' to its correct value before continue
            setIssueException( exceptionState );

               // If 'fallback2Default' is set, check also in 'DEFAULT' section
            if ( getFallback2Default() )
            {

               if( ifExist(variable) )
               {

                  return confData["DEFAULT"][variable].valueComment;

               }
               else
               {

                  return "";

               }

            }
            else
            {

               return "";

            }  // End of 'if ( getFallback2Default() )'

         }  // End of 'if( ifExist(variable, section) )'

      }
      catch (ConfigurationException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'ConfDataReader::getValueDescription()'



      /* Method to check if a given section/variable pair exists.
       *
       * @param variable   Variable name.
       * @param section    Section the variable belongs to.
       *
       */
   bool ConfDataReader::ifExist( string variable,
                                 string section )
      throw(ConfigurationException)
   {

         // Let's make sure that section and variable names are uppercase
      section  = StringUtils::upperCase(section);
      variable = StringUtils::upperCase(variable);

         // Check if section and variable exist
      confMap::const_iterator it;
      it = confData.find(section);
      if( it != confData.end() )
      {
         variableMap::const_iterator it2;
         it2 = (*it).second.find(variable);
         if( it2 != (*it).second.end() )
         {

            // Return the corresponding value, if it exists
            return true;

         }
         else
         {

            if(issueException)
            {

                  // Throw an exception if variable name doesn't exist
               ConfigurationException e(
                                    "Variable '" + variable
                                    + "' in section '" + section
                                    + "' of configuration file '" + filename
                                    + "' does not exist.");

               GPSTK_THROW(e);

            }
            else
            {

               return false;

            }  // End of 'if(issueException)'

         }  // End of 'if( it2 != (*it).second.end() )'

      }
      else
      {

         if(issueException)
         {

               // Check if problem is with section "DEFAULT"
            if ( section == "DEFAULT" )
            {

                  // Throw an exception if section name doesn't exist
               ConfigurationException e(
                                    "Section 'DEFAULT' in configuration file '"
                                    + filename
                                    + "' does not exist. Does file '"
                                    + filename + "' exist?. Do you have "
                                    + "permission to read it?." );

               GPSTK_THROW(e);

            }
            else
            {

                  // Throw an exception if section name doesn't exist
               ConfigurationException e(
                                    "Section '" + section
                                    + "' in configuration file '" + filename
                                    + "' does not exist.");

               GPSTK_THROW(e);

            }  // End of 'if ( section == "DEFAULT" )'

         }
         else
         {

            return false;

         }  // End of 'if(issueException)'

      }  // End of 'if( it != confData.end() )'

   }  // End of method 'ConfDataReader::ifExist()'



      // Method to open AND load configuration data file.
   void ConfDataReader::open(const char* fn)
   {

         // We always open configuration file as "read-only"
      FFTextStream::open(fn, std::ios::in);

      loadData();

      return;

   }  // End of method 'ConfDataReader::open()'



      // Method to get the name of each section in order.
   string ConfDataReader::getEachSection(void)
   {

      if ( itCurrentSection != confData.end() )
      {

            // Store result
         string result( (*itCurrentSection).first );

            // Increment iterator for next time
         ++itCurrentSection;

            // Return result
         return result;

      }
      else
      {
            // If we are at the end, return an empty string
         return "";
      }

   }  // End of method 'ConfDataReader::getEachSection()'



      /* Method to check if the given name is properly formed.
       *
       * @param name    Name to the checked.
       */
   bool ConfDataReader::checkName(string name)
   {

      try
      {

            // Test if first character is alphabetic
         if( StringUtils::isAlphaString(name.substr(0, 1)) )
         {

               // Translate all allowed characters to 'a'
            name = StringUtils::translate( name,"0123456789-_", "a", 'a');

               // If the result is an alphabetic string, the name is valid
            return ( StringUtils::isAlphaString(name) );

         }
         else
         {
               // If first character is not alphabetic, we return false
            return false;
         }

      }
      catch (...)
      {
            // In case of any problem we return false
         return false;
      }

   }  // End of method 'ConfDataReader::checkName()'



}  // End of namespace gpstk
