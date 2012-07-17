/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */



#include "SimpleString.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


static const int DEFAULT_SIZE = 20;

SimpleString::SimpleString ()
: buffer_(new char [1])
{
	buffer_ [0] = '\0';
}


SimpleString::SimpleString (const char *otherBuffer)
: buffer_ (new char [strlen (otherBuffer) + 1])
{
	strcpy (buffer_, otherBuffer);
}

SimpleString::SimpleString (const SimpleString& other)
{
	buffer_ = new char [other.size() + 1];
	strcpy(buffer_, other.buffer_);
}


SimpleString SimpleString::operator= (const SimpleString& other)
{
	delete [] buffer_;
	buffer_ = new char [other.size() + 1];
	strcpy(buffer_, other.buffer_);
	return *this;
}

SimpleString SimpleString::operator+ (const SimpleString& other)
{
	SimpleString ret;
	delete [] ret.buffer_;
	ret.buffer_ = new char [size() + other.size() + 1];
	strcpy(ret.buffer_, buffer_);
	strcat(ret.buffer_, other.buffer_);
	return ret;
}

char *SimpleString::asCharString () const
{
	return buffer_;
}

int SimpleString::size() const
{
	return strlen (buffer_);
}

SimpleString::~SimpleString ()
{
	delete [] buffer_;
}


bool operator== (const SimpleString& left, const SimpleString& right)
{
	return !strcmp (left.asCharString (), right.asCharString ());
}


SimpleString StringFrom (bool value)
{
	char buffer [sizeof ("false") + 1];
	sprintf (buffer, "%s", value ? "true" : "false");
	return SimpleString(buffer);
}

SimpleString StringFrom (const char *value)
{
	return SimpleString(value);
}

SimpleString StringFrom (long value)
{
	char buffer [DEFAULT_SIZE];
	sprintf (buffer, "%ld", value);

	return SimpleString(buffer);
}

SimpleString StringFrom (double value)
{
	char buffer [DEFAULT_SIZE];
	sprintf (buffer, "%lg", value);

	return SimpleString(buffer);
}

SimpleString StringFrom (const SimpleString& value)
{
	return SimpleString(value);
}


