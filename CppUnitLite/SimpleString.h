
///////////////////////////////////////////////////////////////////////////////
//
// SIMPLESTRING.H
//
// One of the design goals of CppUnitLite is to compilation with very old C++
// compilers.  For that reason, I've added a simple string class that provides
// only the operations needed in CppUnitLite.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SIMPLE_STRING
#define SIMPLE_STRING



class SimpleString
{
	friend bool	operator== (const SimpleString& left, const SimpleString& right);

public:
						SimpleString ();
						SimpleString (const char *value);
						SimpleString (const SimpleString& other);
						~SimpleString ();

	SimpleString		operator= (const SimpleString& other);

	char				*asCharString () const;
	int					size() const;

private:
	char				*buffer;
};



SimpleString StringFrom (bool value);
SimpleString StringFrom (const char *value);
SimpleString StringFrom (long value);
SimpleString StringFrom (double value);
SimpleString StringFrom (const SimpleString& other);


#endif
