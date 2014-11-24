/*************************************************************************/
/*! \file getopt.c
\brief Command line parsing 

This file contains a implementation of GNU's Getopt facility. The purpose
for including it here is to ensure portability across different unix- and
windows-based systems.

\warning 
The implementation provided here uses the \c gk_ prefix for all variables
used by the standard Getopt facility to communicate with the program.
So, do read the documentation here.

\verbatim
   Copyright (C) 1987,88,89,90,91,92,93,94,95,96,98,99,2000,2001
   Free Software Foundation, Inc. This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  
\endverbatim
*/
/*************************************************************************/


#include <GKlib.h>

/*************************************************************************/
/* Local function prototypes */
/*************************************************************************/
static void exchange (char **);
static char *gk_getopt_initialize (int, char **, char *);
static int gk_getopt_internal(int argc, char **argv, char *optstring, 
        struct gk_option *longopts, int *longind, int long_only);



/*************************************************************************/
/*! \brief For communication arguments to the caller.

This variable is set by getopt to point at the value of the option argument, 
for those options that accept arguments.
*/
/*************************************************************************/
char *gk_optarg;


/*************************************************************************/
/*! \brief Index in ARGV of the next element to be scanned. 

This variable is set by getopt to the index of the next element of the argv 
array to be processed. Once getopt has found all of the option arguments, 
you can use this variable to determine where the remaining non-option arguments 
begin. 
*/
/*************************************************************************/
int gk_optind = 1; 


/*************************************************************************/
/*! \brief Controls error reporting for unrecognized options.  

If the value of this variable is nonzero, then getopt prints an error 
message to the standard error stream if it encounters an unknown option 
character or an option with a missing required argument. This is the default 
behavior. If you set this variable to zero, getopt does not print any messages,
but it still returns the character ? to indicate an error.
*/
/*************************************************************************/
int gk_opterr = 1;


/*************************************************************************/
/*! \brief Stores unknown option characters

When getopt encounters an unknown option character or an option with a 
missing required argument, it stores that option character in this 
variable. You can use this for providing your own diagnostic messages.
*/
/*************************************************************************/
int gk_optopt = '?';


/*************************************************************************/
/*
Records that the getopt facility has been initialized.
*/
/*************************************************************************/
int gk_getopt_initialized;


/*************************************************************************/
/*
The next char to be scanned in the option-element in which the last option 
character we returned was found.  This allows us to pick up the scan where 
we left off.

If this is zero, or a null string, it means resume the scan by advancing 
to the next ARGV-element.  
*/
/*************************************************************************/
static char *nextchar;


/*************************************************************************/
/*
Value of POSIXLY_CORRECT environment variable.  
*/
/*************************************************************************/
static char *posixly_correct;


/*************************************************************************/
/*
Describe how to deal with options that follow non-option ARGV-elements.

If the caller did not specify anything, the default is REQUIRE_ORDER if 
the environment variable POSIXLY_CORRECT is defined, PERMUTE otherwise.

REQUIRE_ORDER means don't recognize them as options; stop option processing 
when the first non-option is seen.  This is what Unix does.  This mode of 
operation is selected by either setting the environment variable 
POSIXLY_CORRECT, or using `+' as the first character of the list of 
option characters.

PERMUTE is the default.  We permute the contents of ARGV as we scan, so 
that eventually all the non-options are at the end.  This allows options
to be given in any order, even with programs that were not written to
expect this.

RETURN_IN_ORDER is an option available to programs that were written
to expect options and other ARGV-elements in any order and that care 
about the ordering of the two.  We describe each non-option ARGV-element
as if it were the argument of an option with character code 1.
Using `-' as the first character of the list of option characters
selects this mode of operation.

The special argument `--' forces an end of option-scanning regardless
of the value of `ordering'.  In the case of RETURN_IN_ORDER, only
`--' can cause `getopt' to return -1 with `gk_optind' != ARGC.  
*/
/*************************************************************************/
static enum
{
  REQUIRE_ORDER, PERMUTE, RETURN_IN_ORDER
} ordering;



/*************************************************************************/
/* 
Describe the part of ARGV that contains non-options that have
been skipped.  `first_nonopt' is the index in ARGV of the first of them;
`last_nonopt' is the index after the last of them.  
*/
/*************************************************************************/
static int first_nonopt;
static int last_nonopt;





/*************************************************************************/
/*
Handle permutation of arguments.  

Exchange two adjacent subsequences of ARGV. 
One subsequence is elements [first_nonopt,last_nonopt)
which contains all the non-options that have been skipped so far.
The other is elements [last_nonopt,gk_optind), which contains all
the options processed since those non-options were skipped.

`first_nonopt' and `last_nonopt' are relocated so that they describe
the new indices of the non-options in ARGV after they are moved.  
*/
/*************************************************************************/
static void exchange (char **argv)
{
  int bottom = first_nonopt;
  int middle = last_nonopt;
  int top = gk_optind;
  char *tem;

  /* Exchange the shorter segment with the far end of the longer segment.
     That puts the shorter segment into the right place.
     It leaves the longer segment in the right place overall,
     but it consists of two parts that need to be swapped next.  */

  while (top > middle && middle > bottom) {
    if (top - middle > middle - bottom) {
      /* Bottom segment is the short one.  */
      int len = middle - bottom;
      register int i;

      /* Swap it with the top part of the top segment.  */
      for (i = 0; i < len; i++) {
	tem = argv[bottom + i];
	argv[bottom + i] = argv[top - (middle - bottom) + i];
	argv[top - (middle - bottom) + i] = tem;
      }
      /* Exclude the moved bottom segment from further swapping.  */
      top -= len;
    }
    else {
      /* Top segment is the short one.  */
      int len = top - middle;
      register int i;

      /* Swap it with the bottom part of the bottom segment.  */
      for (i = 0; i < len; i++) {
        tem = argv[bottom + i];
        argv[bottom + i] = argv[middle + i];
        argv[middle + i] = tem;
      }
      /* Exclude the moved top segment from further swapping.  */
      bottom += len;
    }
  }

  /* Update records for the slots the non-options now occupy.  */

  first_nonopt += (gk_optind - last_nonopt);
  last_nonopt = gk_optind;
}



/*************************************************************************/
/*
Initialize the internal data when the first call is made.  
*/
/*************************************************************************/
static char *gk_getopt_initialize (int argc, char **argv, char *optstring)
{
  /* Start processing options with ARGV-element 1 (since ARGV-element 0
     is the program name); the sequence of previously skipped
     non-option ARGV-elements is empty.  */

  first_nonopt = last_nonopt = gk_optind;

  nextchar = NULL;

  posixly_correct = getenv("POSIXLY_CORRECT");

  /* Determine how to handle the ordering of options and nonoptions.  */
  if (optstring[0] == '-') {
    ordering = RETURN_IN_ORDER;
    ++optstring;
  }
  else if (optstring[0] == '+') {
    ordering = REQUIRE_ORDER;
    ++optstring;
  }
  else if (posixly_correct != NULL)
    ordering = REQUIRE_ORDER;
  else
    ordering = PERMUTE;

  return optstring;
}


/*************************************************************************/
/*
   Scan elements of ARGV (whose length is ARGC) for option characters
   given in OPTSTRING.

   If an element of ARGV starts with '-', and is not exactly "-" or "--",
   then it is an option element.  The characters of this element
   (aside from the initial '-') are option characters.  If `getopt'
   is called repeatedly, it returns successively each of the option characters
   from each of the option elements.

   If `getopt' finds another option character, it returns that character,
   updating `gk_optind' and `nextchar' so that the next call to `getopt' can
   resume the scan with the following option character or ARGV-element.

   If there are no more option characters, `getopt' returns -1.
   Then `gk_optind' is the index in ARGV of the first ARGV-element
   that is not an option.  (The ARGV-elements have been permuted
   so that those that are not options now come last.)

   OPTSTRING is a string containing the legitimate option characters.
   If an option character is seen that is not listed in OPTSTRING,
   return '?' after printing an error message.  If you set `gk_opterr' to
   zero, the error message is suppressed but we still return '?'.

   If a char in OPTSTRING is followed by a colon, that means it wants an arg,
   so the following text in the same ARGV-element, or the text of the following
   ARGV-element, is returned in `gk_optarg'.  Two colons mean an option that
   wants an optional arg; if there is text in the current ARGV-element,
   it is returned in `gk_optarg', otherwise `gk_optarg' is set to zero.

   If OPTSTRING starts with `-' or `+', it requests different methods of
   handling the non-option ARGV-elements.
   See the comments about RETURN_IN_ORDER and REQUIRE_ORDER, above.

   Long-named options begin with `--' instead of `-'.
   Their names may be abbreviated as long as the abbreviation is unique
   or is an exact match for some defined option.  If they have an
   argument, it follows the option name in the same ARGV-element, separated
   from the option name by a `=', or else the in next ARGV-element.
   When `getopt' finds a long-named option, it returns 0 if that option's
   `flag' field is nonzero, the value of the option's `val' field
   if the `flag' field is zero.

   LONGOPTS is a vector of `struct gk_option' terminated by an
   element containing a name which is zero.

   LONGIND returns the index in LONGOPT of the long-named option found.
   It is only valid when a long-named option has been found by the most
   recent call.

   If LONG_ONLY is nonzero, '-' as well as '--' can introduce
   long-named options.  
*/
/*************************************************************************/
static int gk_getopt_internal(int argc, char **argv, char *optstring, 
        struct gk_option *longopts, int *longind, int long_only)
{
  int print_errors = gk_opterr;
  if (optstring[0] == ':')
    print_errors = 0;

  if (argc < 1)
    return -1;

  gk_optarg = NULL;

  if (gk_optind == 0 || !gk_getopt_initialized) {
    if (gk_optind == 0)
      gk_optind = 1;	/* Don't scan ARGV[0], the program name.  */
      optstring = gk_getopt_initialize (argc, argv, optstring);
      gk_getopt_initialized = 1;
    }

  /* Test whether ARGV[gk_optind] points to a non-option argument.
     Either it does not have option syntax, or there is an environment flag
     from the shell indicating it is not an option.  The later information
     is only used when the used in the GNU libc.  */
# define NONOPTION_P (argv[gk_optind][0] != '-' || argv[gk_optind][1] == '\0')

  if (nextchar == NULL || *nextchar == '\0') {
    /* Advance to the next ARGV-element.  */

    /* Give FIRST_NONOPT & LAST_NONOPT rational values if OPTIND has been
       moved back by the user (who may also have changed the arguments).  */
    if (last_nonopt > gk_optind)
      last_nonopt = gk_optind;
    if (first_nonopt > gk_optind)
      first_nonopt = gk_optind;

    if (ordering == PERMUTE) {
      /* If we have just processed some options following some non-options,
	 exchange them so that the options come first.  */

      if (first_nonopt != last_nonopt && last_nonopt != gk_optind)
	exchange ((char **) argv);
      else if (last_nonopt != gk_optind)
	first_nonopt = gk_optind;

      /* Skip any additional non-options
	 and extend the range of non-options previously skipped.  */

      while (gk_optind < argc && NONOPTION_P)
        gk_optind++;

      last_nonopt = gk_optind;
    }

    /* The special ARGV-element `--' means premature end of options.
       Skip it like a null option,
       then exchange with previous non-options as if it were an option,
       then skip everything else like a non-option.  */

    if (gk_optind != argc && !strcmp (argv[gk_optind], "--")) {
      gk_optind++;

      if (first_nonopt != last_nonopt && last_nonopt != gk_optind)
        exchange ((char **) argv);
      else if (first_nonopt == last_nonopt)
        first_nonopt = gk_optind;
      last_nonopt = argc;

      gk_optind = argc;
    }

    /* If we have done all the ARGV-elements, stop the scan
       and back over any non-options that we skipped and permuted.  */

    if (gk_optind == argc) {
      /* Set the next-arg-index to point at the non-options
	 that we previously skipped, so the caller will digest them.  */
      if (first_nonopt != last_nonopt)
	gk_optind = first_nonopt;
      return -1;
    }

    /* If we have come to a non-option and did not permute it,
       either stop the scan or describe it to the caller and pass it by.  */

    if (NONOPTION_P) {
      if (ordering == REQUIRE_ORDER)
	return -1;
      gk_optarg = argv[gk_optind++];
      return 1;
    }

    /* We have found another option-ARGV-element.
       Skip the initial punctuation.  */

    nextchar = (argv[gk_optind] + 1 + (longopts != NULL && argv[gk_optind][1] == '-'));
  }

  /* Decode the current option-ARGV-element.  */

  /* Check whether the ARGV-element is a long option.

     If long_only and the ARGV-element has the form "-f", where f is
     a valid short option, don't consider it an abbreviated form of
     a long option that starts with f.  Otherwise there would be no
     way to give the -f short option.

     On the other hand, if there's a long option "fubar" and
     the ARGV-element is "-fu", do consider that an abbreviation of
     the long option, just like "--fu", and not "-f" with arg "u".

     This distinction seems to be the most useful approach.  */

  if (longopts != NULL && (argv[gk_optind][1] == '-' || (long_only && (argv[gk_optind][2] || !strchr(optstring, argv[gk_optind][1]))))) {
    char *nameend;
    struct gk_option *p;
    struct gk_option *pfound = NULL;
    int exact = 0;
    int ambig = 0;
    int indfound = -1;
    int option_index;

    for (nameend = nextchar; *nameend && *nameend != '='; nameend++)
      /* Do nothing.  */ ;

    /* Test all long options for either exact match or abbreviated matches.  */
    for (p = longopts, option_index = 0; p->name; p++, option_index++) {
      if (!strncmp (p->name, nextchar, nameend - nextchar)) {
        if ((unsigned int) (nameend - nextchar) == (unsigned int) strlen (p->name)) {
	  /* Exact match found.  */
	  pfound = p;
	  indfound = option_index;
	  exact = 1;
	  break;
	}
	else if (pfound == NULL) {
          /* First nonexact match found.  */
	  pfound = p;
	  indfound = option_index;
	}
	else if (long_only || pfound->has_arg != p->has_arg || pfound->flag != p->flag || pfound->val != p->val)
	  /* Second or later nonexact match found.  */
	  ambig = 1;
      }
    }

    if (ambig && !exact) {
      if (print_errors)
        fprintf(stderr, "%s: option `%s' is ambiguous\n", argv[0], argv[gk_optind]);

      nextchar += strlen (nextchar);
      gk_optind++;
      gk_optopt = 0;
      return '?';
    }

    if (pfound != NULL) {
      option_index = indfound;
      gk_optind++;
      if (*nameend) {
	/* Don't test has_arg with >, because some C compilers don't allow it to be used on enums.  */
	if (pfound->has_arg)
	  gk_optarg = nameend + 1;
	else {
	  if (print_errors) {
	    if (argv[gk_optind - 1][1] == '-')
	      /* --option */
	      fprintf(stderr, "%s: option `--%s' doesn't allow an argument\n", argv[0], pfound->name);
	    else
	      /* +option or -option */
	      fprintf(stderr, "%s: option `%c%s' doesn't allow an argument\n", argv[0], argv[gk_optind - 1][0], pfound->name);
	  }

	  nextchar += strlen (nextchar);

	  gk_optopt = pfound->val;
	  return '?';
	}
      }
      else if (pfound->has_arg == 1) {
	if (gk_optind < argc)
	  gk_optarg = argv[gk_optind++];
	else {
	  if (print_errors)
	    fprintf(stderr, "%s: option `%s' requires an argument\n", argv[0], argv[gk_optind - 1]);
	  nextchar += strlen (nextchar);
	  gk_optopt = pfound->val;
	  return optstring[0] == ':' ? ':' : '?';
	}
      }
      nextchar += strlen (nextchar);
      if (longind != NULL)
        *longind = option_index;
      if (pfound->flag) {
	*(pfound->flag) = pfound->val;
	return 0;
      }
      return pfound->val;
    }

    /* Can't find it as a long option.  If this is not getopt_long_only,
       or the option starts with '--' or is not a valid short
        option, then it's an error. Otherwise interpret it as a short option.  */
    if (!long_only || argv[gk_optind][1] == '-' || strchr(optstring, *nextchar) == NULL) {
      if (print_errors) {
	if (argv[gk_optind][1] == '-')
	  /* --option */
	  fprintf(stderr, "%s: unrecognized option `--%s'\n", argv[0], nextchar);
	else
	  /* +option or -option */
	  fprintf(stderr, "%s: unrecognized option `%c%s'\n", argv[0], argv[gk_optind][0], nextchar);
      }
      nextchar = (char *) "";
      gk_optind++;
      gk_optopt = 0;
      return '?';
    }
  }

  /* Look at and handle the next short option-character.  */
  {
    char c = *nextchar++;
    char *temp = strchr(optstring, c);

    /* Increment `gk_optind' when we start to process its last character.  */
    if (*nextchar == '\0')
      ++gk_optind;

    if (temp == NULL || c == ':') {
      if (print_errors) {
        if (posixly_correct)
	  /* 1003.2 specifies the format of this message.  */
	  fprintf(stderr, "%s: illegal option -- %c\n", argv[0], c);
	else
	  fprintf(stderr, "%s: invalid option -- %c\n", argv[0], c);
      }
      gk_optopt = c;
      return '?';
    }

    /* Convenience. Treat POSIX -W foo same as long option --foo */
    if (temp[0] == 'W' && temp[1] == ';') {
      char *nameend;
      struct gk_option *p;
      struct gk_option *pfound = NULL;
      int exact = 0;
      int ambig = 0;
      int indfound = 0;
      int option_index;

      /* This is an option that requires an argument.  */
      if (*nextchar != '\0') {
	gk_optarg = nextchar;
	/* If we end this ARGV-element by taking the rest as an arg,
	   we must advance to the next element now.  */
	gk_optind++;
      }
      else if (gk_optind == argc) {
	if (print_errors) {
	  /* 1003.2 specifies the format of this message.  */
	  fprintf(stderr, "%s: option requires an argument -- %c\n", argv[0], c);
	}
	gk_optopt = c;
	if (optstring[0] == ':')
	  c = ':';
	else
	  c = '?';
	return c;
      }
      else
	/* We already incremented `gk_optind' once; increment it again when taking next ARGV-elt as argument.  */
	gk_optarg = argv[gk_optind++];

      /* gk_optarg is now the argument, see if it's in the table of longopts.  */

      for (nextchar = nameend = gk_optarg; *nameend && *nameend != '='; nameend++)
	/* Do nothing.  */ ;

      /* Test all long options for either exact match or abbreviated matches.  */
      for (p = longopts, option_index = 0; p->name; p++, option_index++) {
	if (!strncmp (p->name, nextchar, nameend - nextchar)) {
	  if ((unsigned int) (nameend - nextchar) == strlen (p->name)) {
	    /* Exact match found.  */
	    pfound = p;
	    indfound = option_index;
	    exact = 1;
	    break;
	  }
	  else if (pfound == NULL) {
	    /* First nonexact match found.  */
	    pfound = p;
	    indfound = option_index;
	  }
	  else
	    /* Second or later nonexact match found.  */
	    ambig = 1;
	}
      }
      if (ambig && !exact) {
	if (print_errors)
	  fprintf(stderr, "%s: option `-W %s' is ambiguous\n", argv[0], argv[gk_optind]);
	nextchar += strlen (nextchar);
	gk_optind++;
	return '?';
      }
      if (pfound != NULL) {
	option_index = indfound;
	if (*nameend) {
	  /* Don't test has_arg with >, because some C compilers don't allow it to be used on enums.  */
	  if (pfound->has_arg)
	    gk_optarg = nameend + 1;
	  else {
	    if (print_errors)
	      fprintf(stderr, "%s: option `-W %s' doesn't allow an argument\n", argv[0], pfound->name);

	    nextchar += strlen (nextchar);
	    return '?';
	  }
	}
	else if (pfound->has_arg == 1) {
	  if (gk_optind < argc)
	    gk_optarg = argv[gk_optind++];
	  else {
	    if (print_errors)
	      fprintf(stderr, "%s: option `%s' requires an argument\n", argv[0], argv[gk_optind - 1]);
	    nextchar += strlen (nextchar);
	    return optstring[0] == ':' ? ':' : '?';
	  }
        }
	nextchar += strlen (nextchar);
	if (longind != NULL)
	  *longind = option_index;
	if (pfound->flag) {
	  *(pfound->flag) = pfound->val;
	  return 0;
	}
	return pfound->val;
      }
      nextchar = NULL;
      return 'W';	/* Let the application handle it.   */
    }

    if (temp[1] == ':') {
      if (temp[2] == ':') {
	/* This is an option that accepts an argument optionally.  */
	if (*nextchar != '\0') {
  	  gk_optarg = nextchar;
	  gk_optind++;
	}
	else
	  gk_optarg = NULL;
	nextchar = NULL;
      }
      else {
	/* This is an option that requires an argument.  */
	if (*nextchar != '\0') {
	  gk_optarg = nextchar;
	  /* If we end this ARGV-element by taking the rest as an arg, we must advance to the next element now.  */
	  gk_optind++;
	}
	else if (gk_optind == argc) {
	  if (print_errors) {
	    /* 1003.2 specifies the format of this message.  */
	    fprintf(stderr, "%s: option requires an argument -- %c\n", argv[0], c);
	  }
	  gk_optopt = c;
	  if (optstring[0] == ':')
	    c = ':';
	  else
	    c = '?';
	}
	else
	  /* We already incremented `gk_optind' once; increment it again when taking next ARGV-elt as argument.  */
	  gk_optarg = argv[gk_optind++];
	  nextchar = NULL;
      }
    }
    return c;
  }
}



/*************************************************************************/
/*! \brief Parse command-line arguments

The gk_getopt() function gets the next option argument from the argument 
list specified by the \c argv and \c argc arguments. Normally these values 
come directly from the arguments received by main().

\param argc is the number of command line arguments passed to main().
\param argv is an array of strings storing the above command line 
       arguments.
\param options is a string that specifies the option characters that 
       are valid for this program. An option character in this string 
       can be followed by a colon (`:') to indicate that it takes a 
       required argument. If an option character is followed by two 
       colons (`::'), its argument is optional; this is a GNU extension.

\return  
It returns the option character for the next command line option. When no 
more option arguments are available, it returns -1. There may still be 
more non-option arguments; you must compare the external variable 
#gk_optind against the \c argc parameter to check this.

\return  
If the option has an argument, gk_getopt() returns the argument by storing 
it in the variable #gk_optarg. You don't ordinarily need to copy the 
#gk_optarg string, since it is a pointer into the original \c argv array, 
not into a static area that might be overwritten.

\return  
If gk_getopt() finds an option character in \c argv that was not included 
in options, or a missing option argument, it returns `?' and sets the 
external variable #gk_optopt to the actual option character. 
If the first character of options is a colon (`:'), then gk_getopt() 
returns `:' instead of `?' to indicate a missing option argument. 
In addition, if the external variable #gk_opterr is nonzero (which is 
the default), gk_getopt() prints an error message.  This variable is 
set by gk_getopt() to point at the value of the option argument, 
for those options that accept arguments.


gk_getopt() has three ways to deal with options that follow non-options 
\c argv elements. The special argument <tt>`--'</tt> forces in all cases 
the end of option scanning.
  - The default is to permute the contents of \c argv while scanning it 
    so that eventually all the non-options are at the end. This allows 
    options to be given in any order, even with programs that were not 
    written to expect this.
  - If the options argument string begins with a hyphen (`-'), this is 
    treated specially. It permits arguments that are not options to be 
    returned as if they were associated with option character `\\1'.
  - POSIX demands the following behavior: The first non-option stops 
    option processing. This mode is selected by either setting the 
    environment variable POSIXLY_CORRECT or beginning the options
    argument string with a plus sign (`+'). 

*/
/*************************************************************************/
int gk_getopt(int argc, char **argv, char *options)
{
  return gk_getopt_internal(argc, argv, options, NULL, NULL, 0);
}


/*************************************************************************/
/*! \brief Parse command-line arguments with long options

This function accepts GNU-style long options as well as single-character 
options. 

\param argc is the number of command line arguments passed to main().
\param argv is an array of strings storing the above command line 
       arguments.
\param options describes the short options to accept, just as it does 
       in gk_getopt(). 
\param long_options describes the long options to accept. See the 
       defintion of ::gk_option for more information.
\param opt_index this is a returned variable.  For any long option, 
       gk_getopt_long() tells you the index in the array \c long_options 
       of the options definition, by storing it into <tt>*opt_index</tt>. 
       You can get the name of the option with <tt>longopts[*opt_index].name</tt>. 
       So you can distinguish among long options either by the values 
       in their val fields or by their indices. You can also distinguish 
       in this way among long options that set flags.


\return
When gk_getopt_long() encounters a short option, it does the same thing 
that gk_getopt() would do: it returns the character code for the option, 
and stores the options argument (if it has one) in #gk_optarg.

\return
When gk_getopt_long() encounters a long option, it takes actions based 
on the flag and val fields of the definition of that option.

\return
If flag is a null pointer, then gk_getopt_long() returns the contents 
of val to indicate which option it found. You should arrange distinct 
values in the val field for options with different meanings, so you 
can decode these values after gk_getopt_long() returns. If the long 
option is equivalent to a short option, you can use the short option's 
character code in val.

\return
If flag is not a null pointer, that means this option should just set 
a flag in the program. The flag is a variable of type int that you 
define. Put the address of the flag in the flag field. Put in the 
val field the value you would like this option to store in the flag. 
In this case, gk_getopt_long() returns 0.

\return
When a long option has an argument, gk_getopt_long() puts the argument 
value in the variable #gk_optarg before returning. When the option has 
no argument, the value in #gk_optarg is a null pointer. This is
how you can tell whether an optional argument was supplied.

\return
When gk_getopt_long() has no more options to handle, it returns -1, 
and leaves in the variable #gk_optind the index in argv of the next 
remaining argument. 
*/
/*************************************************************************/
int gk_getopt_long( int argc, char **argv, char *options, 
       struct gk_option *long_options, int *opt_index)
{
  return gk_getopt_internal (argc, argv, options, long_options, opt_index, 0);
}



/*************************************************************************/
/*! \brief Parse command-line arguments with only long options

Like gk_getopt_long(), but '-' as well as '--' can indicate a long option.
If an option that starts with '-' (not '--') doesn't match a long option,
but does match a short option, it is parsed as a short option instead.  
*/
/*************************************************************************/
int gk_getopt_long_only(int argc, char **argv, char *options, 
       struct gk_option *long_options, int *opt_index)
{
  return gk_getopt_internal(argc, argv, options, long_options, opt_index, 1);
}

