#pragma ident "$Id$"

// expandtilde.cpp Expand tilde (~) in filenames.

#include <iostream>
#include <fstream>
#include "expandtilde.hpp"
#include "StringUtils.hpp"

using namespace std;
using namespace gpstk;

void expand_filename(string& filename)
{
#ifndef _WIN32
   static char *chome = getenv("HOME");
   static string home = string(chome);

   // assume tilde occurs only once
   string::size_type pos = filename.find_first_of("~");
   if(pos == string::npos) return;
   string newname;
   if(pos > 0) newname = filename.substr(0,pos);
   filename = filename.substr(pos+1);
   StringUtils::stripLeading(filename,"/");
   StringUtils::stripTrailing(home,"/");
   newname += home + string("/") + filename;
   filename = newname;
#endif
}

void expand_filename(vector<string>& sarray)
{
   for(size_t i=0; i<sarray.size(); i++) expand_filename(sarray[i]);
}

void include_path(string path, string& file)
{
   if(!path.empty()) {
      StringUtils::stripTrailing(path,"/");
      StringUtils::stripTrailing(path,"\\");
      file = path + string("/") + file;
   }
}

void include_path(string path, vector<string>& sarray)
{
   if(!path.empty()) {
      StringUtils::stripTrailing(path,"/");
      StringUtils::stripTrailing(path,"\\");
      for(size_t i=0; i<sarray.size(); i++)
         sarray[i] = path + string("/") + sarray[i];
   }
}

// return false if file cannot be opened
bool expand_list_file(string& filename, vector<string>& values)
{
   string line,word;
   // DO NOT clear values, add to it

   // open list file
   ifstream infile;
   infile.open(filename.c_str());
   if(!infile.is_open()) return false;

   // read the list file
   while(1) {
      getline(infile,line);
      StringUtils::stripTrailing(line,'\r');
      StringUtils::stripLeading(line);
      while(!line.empty()) {
         word = StringUtils::stripFirstWord(line);
         if(word.substr(0,1) == "#") break;        // skip '#...' to end of line
         values.push_back(word);
      }
      if(infile.eof() || !infile.good()) break;
   }

   infile.close();

   return true;
}
