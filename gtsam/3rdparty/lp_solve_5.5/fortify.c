/*
 * FILE:
 *   fortify.c
 *
 * DESCRIPTION:
 *     A fortified shell for malloc, realloc, calloc, strdup, getcwd, tempnam
 *     and free.
 *     To use Fortify, each source file will need to #include "fortify.h".  To
 * enable  Fortify,  define the symbol FORTIFY.  If FORTIFY is not defined, it
 * will compile away to nothing.  If you do not have stderr available, you may
 * wish  to  set  an  alternate output function.  See _Fortify_SetOutputFunc(),
 * below.
 *     You will also need to link in fortify.o
 *
 *     None of the functions in this file should really be called
 *   directly; they really should be called through the macros
 *   defined in fortify.h
 *
 */

#if defined FORTIFY

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdlib.h>

#if defined MSDOS || defined __BORLANDC__ || defined WIN32 || defined __HIGHC__
# include <direct.h>
#endif

extern int EndOfPgr(int);

#define __FORTIFY_C__ /* So fortify.h knows to not define the fortify macros */
#include "fortify.h"

#include "ufortify.h" /* the user's options */

char *_Fortify_file=NULL;
int _Fortify_line=0;

#ifndef FORTIFY_TRANSPARENT

#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>

#if defined MSDOS || defined __BORLANDC__ || defined WIN32 || defined __HIGHC__
# if !defined WIN32
#  undef MSDOS
#  define MSDOS
# endif
# include <conio.h>
#else
# include <unistd.h>
# include <termio.h>
# define getch() getchar()
#endif

#if defined _WINDOWS
# include "windows.h"
# if !defined WIN32
#  include "toolhelp.h"
# endif
#endif

#if defined LONGNAME
# include "longname.h"
#endif

#if !defined MIN
# define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

struct Header
{
	char          *File;   /* The sourcefile of the caller   */
	unsigned short Line;   /* The sourceline of the caller   */
	size_t         Size;   /* The size of the malloc'd block */
	struct Header *Prev,   /* List pointers                  */
		      *Next;
	int            Checksum;  /* For validating the Header structure; see ChecksumHeader() */
	unsigned char  Scope;
};

#if defined AViiON || defined __GNUC__ || defined _MSC_VER
# define _static static
#else
# define _static
#endif

_static char *address __OF((void *addr));
_static int TimeToCheck __OF((void));
_static int CheckBlock __OF((struct Header *h, char *file, unsigned long line));
_static int CheckPointer __OF((unsigned char *ptr, unsigned long size, char *file, unsigned long line));
_static int CheckFortification __OF((unsigned char *ptr, unsigned char value, size_t size));
_static void SetFortification __OF((unsigned char *ptr, unsigned char value, size_t size));
_static void OutputFortification __OF((unsigned char *ptr, unsigned char value, size_t size));
_static int IsHeaderValid __OF((struct Header *h));
_static void MakeHeaderValid __OF((struct Header *h));
_static int ChecksumHeader __OF((struct Header *h));
_static int IsOnList __OF((struct Header *h));
_static void OutputHeader __OF((struct Header *h));
_static void OutputMemory __OF((struct Header *h));
_static void st_DefaultOutput __OF((char *String));
_static void WaitIfstdOutput __OF((void));

static char stdOutput = 0;        /* If true, did some stderr output */
static OutputFuncPtr  st_Output = st_DefaultOutput; /* Output function for errors */

#if !defined MSDOS && !defined WIN32
static int strnicmp(s1,s2,maxlen)
 char *s1,*s2;
 size_t maxlen;
 {
  while ((maxlen) && (*s1) && (*s2) && (toupper(*s1)==toupper(*s2))) {
   maxlen--;
   s1++;
   s2++;
  }
  return((maxlen) ? toupper(*s1)-toupper(*s2) : 0);
 }

static int stricmp(s1,s2)
 char *s1,*s2;
 {
  return(strnicmp(s1,s2,strlen(s1)+1));
 }
#endif

static char *address(void *addr)
{
        static char str[80];

#if defined KNOWS_POINTER_TYPE
        sprintf(str,"%p",addr);
#else
        sprintf(str,"%lx",(unsigned long) addr);
#endif
        return(str);
}

#ifdef FORTIFY_CheckInterval
int TimeToCheck()
{
        static time_t lastcheck=0L;
	time_t t;
        int ret = 0;

	time(&t);
	if ((lastcheck==0L) || (t-lastcheck>=FORTIFY_CheckInterval))
        {
                lastcheck = t;
                ret = 1;
        }
        return(ret);
}
#endif

static FILE *gfile=NULL;
static int Nchars=0,Nlines=0;
static char flag=0;

static void _Fortify_NoOutput()
 {
 }

static void st_DefaultOutput(char *String)
{
        static FILE *file;
        static char first=1;

        if (first) {
                file=stderr;
                first=0;
        }

        if (stdOutput==0) {
        	Nchars=Nlines=0;
                if (gfile!=NULL) rewind(gfile);
        }

        if (flag==0)
        {
                char *ptr;

        	file=stderr;
                flag = 1;
                if ((ptr=getenv("FORTIFY_OUTPUT"))!=NULL)
                {
                        if ((stricmp(ptr,"null")==0) || (stricmp(ptr,"nul")==0))
                                file=NULL;
                        else if (stricmp(ptr,"stderr")==0)
                                file=stderr;
                        else if (stricmp(ptr,"stdout")==0)
                                file=stdout;
#if defined MSDOS && !defined _WINDOWS && !defined WIN32
                        else if (stricmp(ptr,"stdprn")==0)
                                file=stdprn;
#endif
                        else if ((file=fopen(ptr,"w"))==NULL)
                        {
#if !defined _WINDOWS
                                fprintf(stderr,"\r\nFortify: Unable to create logfile %s\r\n",ptr);
				EndOfPgr(4);
#else
    				{
    					char str[255];

    					sprintf(str,"Unable to create logfile\n \"%s\"",ptr);
    					MessageBox((HWND) NULL,(LPCSTR) str,(LPCSTR) "Fortify",(UINT) MB_ICONSTOP);
#if 0
#if defined WIN32
				        /* TerminateProcess(GetCurrentProcess(),65535); */
                                        ExitProcess(65535);
#else
    					TerminateApp((HTASK) NULL,(WORD) NO_UAE_BOX);
#endif
#else
                                        EndOfPgr(1);
#endif
    				}
#endif
                        }
		}
		if ((file!=NULL) && (file!=stderr) && (file!=stdout))
		{
			time_t t;

			time(&t);
			fprintf(file,"Generated on: %s%s\n",
                                ctime(&t),
                                (file==stdout) || (file==stderr) ? "\r" : ""
                               );
		}
	}
	if (file!=NULL)
	{
#if defined _WINDOWS
                if ((file==stdout) || (file==stderr)) {
#if defined LINE_BY_LINE
                	if (MessageBox((HWND) NULL,(LPCSTR) String,(LPCSTR) "Fortify",(UINT) MB_OKCANCEL /* |MB_ICONINFORMATION */)==IDCANCEL)
#if 0
#if defined WIN32
                         /* TerminateProcess(GetCurrentProcess(),65535); */
                         ExitProcess(65535);
#else
    			 TerminateApp((HTASK) NULL,(WORD) NO_UAE_BOX);
#endif
#else
                         EndOfPgr(1);
#endif
#else
			{
                        	char *ptr;

                        	ptr="fortify.tmp";
				if ((ptr==NULL) || ((file=gfile=fopen(ptr,"w+"))==NULL))
                        	{
    					char str[255];

    					sprintf(str,"Unable to create temporary file\n \"%s\"",(ptr==NULL) ? "(NULL)" : ptr);
    					MessageBox((HWND) NULL,(LPCSTR) str,(LPCSTR) "Fortify",(UINT) MB_ICONSTOP);
#if 0
#if defined WIN32
                                        /* TerminateProcess(GetCurrentProcess(),65535); */
                                        ExitProcess(65535);
#else
    					TerminateApp((HTASK) NULL,(WORD) NO_UAE_BOX);
#endif
#else
                                        EndOfPgr(1);
#endif
                        	}
                    	}
#endif
                }
                if ((file!=stdout) && (file!=stderr))
#endif
                        {
                                int i,ch=-1;

                        	for (i=0;(String[i]) && (Nlines<30);i++)
                                 if (String[i]=='\n') Nlines++;
				if ((String[i]) && (String[i+1])) {
					ch=String[i+1];
					String[i+1]=0;
				}
                                if ((file==stdout) || (file==stderr)) {
                                 char *ptr=String;
                                 int i;

                                 do {
                                  for (i=0;(ptr[i]) && (ptr[i]!='\r') && (ptr[i]!='\n');i++);
                                  Nchars+=fprintf(file,"%-*.*s%s",
                                                  i,i,
                                                  ptr,
                                                  (ptr[i]) ? "\r\n" : ""
                                                 );
                                  ptr+=i;
                                  if (ptr[0]=='\r') ptr++;
                                  if (ptr[0]=='\n') ptr++;
                                 } while (*ptr);
                                }
                                else Nchars+=fprintf(file,String);
				if (ch>=0) String[i+1]=(char)ch;
				if (Nlines>=30) {
                                	WaitIfstdOutput();
                                    	Nchars=Nlines=0;
                                    	stdOutput = 0;
                                        if ((String[i]) && (String[i+1])) {
                                                if ((file==stderr) || (file==stdout) || ((gfile!=NULL) && (Nchars)))
                                                	stdOutput = 1;
                                         	st_DefaultOutput(String+i);
                                        }
                                }
                        }
                if ((file==stderr) || (file==stdout) || ((gfile!=NULL) && (Nchars)))
                	stdOutput = 1;
	}
}

static void WaitIfstdOutput()
{
#if !defined _WINDOWS
        if((stdOutput) && (st_Output != (OutputFuncPtr) _Fortify_NoOutput))
        {
#ifdef FORTIFY_WAIT_FOR_KEY
                static signed char wait_on_key=-1;

                if(wait_on_key<0)
		{
			char *ptr;

                        if (((ptr=getenv("FORTIFY_WAIT_FOR_KEY"))!=NULL) &&
			    (tolower(*ptr)=='n')) wait_on_key = 0;
                        else wait_on_key = 1;

                }
		if(wait_on_key)
		{
                        char c;

#if !defined MSDOS && !defined WIN32
                        struct termio tio,tiobak;
                        char flag;

                        if ((flag=ioctl(0,TCGETA,&tio))==0) /* handle 0 is stdin */
                        {
                                tiobak=tio;
                        	tio.c_lflag&=~ICANON;
                                tio.c_lflag&=~ECHO;
                        	tio.c_cc[VMIN]=1;
                        	ioctl(0,TCSETA,&tio);
	                }
#endif /* !MSDOS */
			c = (char)getch();

#if !defined MSDOS && !defined WIN32
                        if (flag==0)
                        	ioctl(0,TCSETA,&tiobak);
#endif /* !MSDOS */

			if ((c == 3) || (c == 0x1b)) EndOfPgr(3);
		}
#endif /* FORTIFY_WAIT_FOR_KEY */

        }
#else
# if !defined LINE_BY_LINE
        if ((stdOutput) && (gfile!=NULL) && (Nchars))
        {
                char *ptr;

                ptr=malloc(Nchars+1);
                if (ptr!=NULL)
                {
                        int n=0,l=0;

                        rewind(gfile);
                        while ((n<Nchars) && (l<Nlines))
                        {
                                fgets(ptr+n,Nchars-n+1,gfile);
                                n+=(int)strlen(ptr+n);
                                l++;
                        }
                	if (MessageBox((HWND) NULL,(LPCSTR) ptr,(LPCSTR) "Fortify",(UINT) MB_OKCANCEL /* |MB_ICONINFORMATION */)==IDCANCEL)
#if 0
#if defined WIN32
                         /* TerminateProcess(GetCurrentProcess(),65535); */
                         ExitProcess(65535);
#else
    			 TerminateApp((HTASK) NULL,(WORD) NO_UAE_BOX);
#endif
#else
                         EndOfPgr(1);
#endif
                }
                free(ptr);
        }
# endif
#endif
	stdOutput = 0;
}

static struct Header *st_Head = NULL; /* Head of alloc'd memory list */
static char st_Buffer[256];       /* Temporary buffer for sprintf's */
static int st_Disabled = 0;       /* If true, Fortify is inactive */
static int st_MallocFailRate = 0; /* % of the time to fail mallocs */

static char          *st_LastVerifiedFile = "unknown";
static unsigned short st_LastVerifiedLine = 0;
static unsigned char  st_Scope            = 0;
static void           OutputLastVerifiedPoint __OF((void));

void FORTIFY_STORAGE
_Fortify_Init(char *file,unsigned long line)
{
	if (gfile!=NULL) fclose(gfile);
	gfile=NULL;
	Nchars=Nlines=0;
        flag=0;
        st_Head=NULL;
        stdOutput=0;
        st_Output=st_DefaultOutput;
        st_Disabled=0;
        st_MallocFailRate=0;
        st_LastVerifiedFile="unknown";
        st_LastVerifiedLine=0;
        st_Scope=0;
}


/*
 * _Fortify_malloc() - Allocates a block of memory, with extra bits for
 *                    misuse protection/detection.
 *
 *    Features:
 *     +  Adds the malloc'd memory onto Fortify's own private list.
 *        (With a checksum'd header to detect corruption of the memory list)
 *     +  Places sentinals on either side of the user's memory with
 *        known data in them, to detect use outside of the bounds
 *        of the block
 *     +  Initializes the malloc'd memory to some "nasty" value, so code
 *        can't rely on it's contents.
 *     +  Can check all sentinals on every malloc.
 *     +  Can generate a warning message on a malloc fail.
 *     +  Can randomly "fail" at a set fail rate
 */
void *FORTIFY_STORAGE
_Fortify_malloc(size_t size,char *file,unsigned long line)
{
	unsigned char *ptr;
	struct Header *h;

        stdOutput = 0;

	FORTIFY_LOCK();

	if(st_Disabled)
	{
		ptr = (unsigned char *) malloc(size);
		FORTIFY_UNLOCK();
                WaitIfstdOutput();
		return((void *) ptr);
	}

#ifdef CHECK_ALL_MEMORY_ON_MALLOC
#ifdef FORTIFY_CheckInterval
        if (TimeToCheck())
#endif
		_Fortify_CheckAllMemory(file, line);
#endif

	if(size == 0)
	{
#ifdef WARN_ON_ZERO_MALLOC
    	sprintf(st_Buffer,
        	    "\nFortify: %s.%ld\n         malloc(0) attempted failed\n",
            	file, line);
		st_Output(st_Buffer);
#endif

		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	if(st_MallocFailRate > 0)
	{
		if(rand() % 100 < st_MallocFailRate)
		{
#ifdef WARN_ON_FALSE_FAIL
			sprintf(st_Buffer,
					"\nFortify: %s.%ld\n         malloc(%ld) \"false\" failed\n",
							file, line, (unsigned long)size);
			st_Output(st_Buffer);
#endif
			FORTIFY_UNLOCK();
                        WaitIfstdOutput();
                        return(0);
		}
	}

	/*
	 * malloc the memory, including the space for the header and fortification
	 * buffers
	 */
#ifdef WARN_ON_SIZE_T_OVERFLOW
	{
		size_t private_size = sizeof(struct Header)
							+ FORTIFY_BEFORE_SIZE + size + FORTIFY_AFTER_SIZE;

		if(private_size < size) /* Check to see if the added baggage is larger than size_t */
		{
			sprintf(st_Buffer,
					"\nFortify: %s.%ld\n         malloc(%ld) has overflowed size_t.\n",
					file, line, (unsigned long)size);
			st_Output(st_Buffer);
			FORTIFY_UNLOCK();
			WaitIfstdOutput();
			return(0);
		}
	}
#endif

	ptr = (unsigned char *) malloc(sizeof(struct Header) +
				 FORTIFY_BEFORE_SIZE + size + FORTIFY_AFTER_SIZE);
	if(!ptr)
	{
#ifdef WARN_ON_MALLOC_FAIL
		sprintf(st_Buffer, "\nFortify: %s.%ld\n         malloc(%ld) failed\n",
				file, line, (unsigned long)size);
		st_Output(st_Buffer);
#endif

		FORTIFY_UNLOCK();
                WaitIfstdOutput();
                return(0);
	}

	/*
	 * Initialize and validate the header
	 */
	h = (struct Header *)ptr;

	h->Size = size;

	h->File = file;
	h->Line = (unsigned short) line;

	h->Next = st_Head;
	h->Prev = 0;

	h->Scope = st_Scope;

	if(st_Head)
	{
		st_Head->Prev = h;
		MakeHeaderValid(st_Head);
	}

	st_Head = h;

	MakeHeaderValid(h);


	/*
	 * Initialize the fortifications
	 */
	SetFortification(ptr + sizeof(struct Header),
	                 FORTIFY_BEFORE_VALUE, FORTIFY_BEFORE_SIZE);
	SetFortification(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE + size,
	                 FORTIFY_AFTER_VALUE, FORTIFY_AFTER_SIZE);

#ifdef FILL_ON_MALLOC
	/*
	 * Fill the actual user memory
	 */
	SetFortification(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE,
	                 FILL_ON_MALLOC_VALUE, size);
#endif

	/*
	 * We return the address of the user's memory, not the start of the block,
	 * which points to our magic cookies
	 */

	FORTIFY_UNLOCK();
        WaitIfstdOutput();
	return((void *) (ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE));
}

/*
 * _Fortify_free() - This free must be used for all memory allocated with
 *                  _Fortify_malloc().
 *
 *   Features:
 *     + Pointers are validated before attempting a free - the pointer
 *       must point to a valid malloc'd bit of memory.
 *     + Detects attempts at freeing the same block of memory twice
 *     + Can clear out memory as it is free'd, to prevent code from using
 *       the memory after it's been freed.
 *     + Checks the sentinals of the memory being freed.
 *     + Can check the sentinals of all memory.
 */

void FORTIFY_STORAGE
_Fortify_free(void *uptr,char *file,unsigned long line)
{
	unsigned char *ptr = (unsigned char *)uptr - sizeof(struct Header) - FORTIFY_BEFORE_SIZE;
	struct Header *h = (struct Header *)ptr;

        stdOutput = 0;

	FORTIFY_LOCK();

	if(st_Disabled)
	{
		free(uptr);
		FORTIFY_UNLOCK();
                WaitIfstdOutput();
                return;
	}

#ifdef CHECK_ALL_MEMORY_ON_FREE
#ifdef FORTIFY_CheckInterval
        if (TimeToCheck())
#endif
		_Fortify_CheckAllMemory(file, line);
#endif

#ifdef PARANOID_FREE
	if(!IsOnList(h))
	{
		sprintf(st_Buffer,
		 "\nFortify: %s.%ld\n         Invalid pointer, corrupted header, or possible free twice\n",
				file, line);
		st_Output(st_Buffer);
		OutputLastVerifiedPoint();
		goto fail;
	}
#endif

	if(!CheckBlock(h, file, line))
		goto fail;

	/*
	 * Remove the block from the list
	 */
	if(h->Prev)
	{
		if(!CheckBlock(h->Prev, file, line))
			goto fail;

		h->Prev->Next = h->Next;
		MakeHeaderValid(h->Prev);
	}
	else
		st_Head = h->Next;

	if(h->Next)
	{
		if(!CheckBlock(h->Next, file, line))
			goto fail;

		h->Next->Prev = h->Prev;
		MakeHeaderValid(h->Next);
	}

#ifdef FILL_ON_FREE
	/*
	 * Nuke out all memory that is about to be freed
	 */
	SetFortification(ptr, FILL_ON_FREE_VALUE,
	                 sizeof(struct Header) + FORTIFY_BEFORE_SIZE + h->Size + FORTIFY_AFTER_SIZE);
#endif

	/*
	 * And do the actual free
	 */
	free(ptr);
	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return;

fail:
	sprintf(st_Buffer, "         free(%s) failed\n", address(uptr));
	st_Output(st_Buffer);
	FORTIFY_UNLOCK();
        WaitIfstdOutput();
}

/*
 * _Fortify_realloc() - Uses _Fortify_malloc() and _Fortify_free() to implement
 *                     realloc().
 *
 *   Features:
 *        + The realloc'd block is ALWAYS moved.
 *        + The pointer passed to realloc() is verified in the same way that
 *          _Fortify_free() verifies pointers before it frees them.
 *        + All the _Fortify_malloc() and _Fortify_free() protection
 */
void *FORTIFY_STORAGE
_Fortify_realloc(void *ptr,size_t new_size,char *file,unsigned long line)
{
	void *new_ptr;
	struct Header *h;

	if(new_size == 0)
	{
		_Fortify_free(ptr,file,line);
		return(NULL);
	}

	h = (struct Header *)((unsigned char *)ptr - sizeof(struct Header) - FORTIFY_BEFORE_SIZE);

        stdOutput = 0;

	if(st_Disabled)
	{
		FORTIFY_LOCK();
		new_ptr = (void *) realloc(ptr, new_size);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
                return(new_ptr);
	}

	if(!ptr)
        {
		void *FORTIFY_STORAGE ret = _Fortify_malloc(new_size, file, line);

                WaitIfstdOutput();
		return(ret);
        }

	FORTIFY_LOCK();

	if(!IsOnList(h))
	{
		sprintf(st_Buffer,
				"\nFortify: %s.%ld\n         Invalid pointer or corrupted header passed to realloc\n",
				file, line);
		st_Output(st_Buffer);
		goto fail;
	}

	if(!CheckBlock(h, file, line))
		goto fail;

	new_ptr = _Fortify_malloc(new_size, file, line);
	if(!new_ptr)
	{
		FORTIFY_UNLOCK();
                WaitIfstdOutput();
                return(0);
	}

	if(h->Size < new_size)
		memcpy(new_ptr, ptr, h->Size);
	else
		memcpy(new_ptr, ptr, new_size);

	_Fortify_free(ptr, file, line);
	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(new_ptr);

fail:
	sprintf(st_Buffer, "         realloc(%s, %ld) failed\n", address(ptr), (unsigned long)new_size);
	st_Output(st_Buffer);
	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(NULL);
}


/*
 * __Fortify_CheckPointer() - Returns true if the uptr points to a valid
 *   piece of _Fortify_malloc()'d memory. The memory must be on the malloc'd
 *   list, and it's sentinals must be in tact.
 *     If anything is wrong, an error message is issued.
 *
 *   (Note - if fortify is disabled, this function always returns true).
 */
static int FORTIFY_STORAGE
__Fortify_CheckPointer(void *uptr,char OnlyStart,unsigned long size,char *file,unsigned long line)
{
	unsigned char *ptr = (unsigned char *)uptr - sizeof(struct Header) - FORTIFY_BEFORE_SIZE;
	int r = 1, StartPointer;

        stdOutput = 0;

	if(st_Disabled)
        {
                WaitIfstdOutput();
		return(1);
        }

	FORTIFY_LOCK();

	StartPointer = IsOnList((struct Header *)ptr);
	if((OnlyStart) && (!StartPointer))
	{
		sprintf(st_Buffer,
		       "\nFortify: %s.%ld\n         Invalid pointer or corrupted header detected (%s)\n",
		       file, line, address(uptr));
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
                WaitIfstdOutput();
                return(0);
	}

	if((OnlyStart) || (StartPointer))
		r = CheckBlock((struct Header *)ptr, file, line);
	if(!OnlyStart)
		r = CheckPointer((unsigned char *)uptr, size, file, line);
	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(r);
}


int FORTIFY_STORAGE
_Fortify_CheckPointer(void *uptr,char *file,unsigned long line)
{
	return(__Fortify_CheckPointer(uptr,1,0,file,line));
}

/*
 * Fortify_SetOutputFunc(OutputFuncPtr Output) - Sets the function used to
 *   output all error and diagnostic messages by fortify. The output function
 *   takes a single unsigned char * argument, and must be able to handle newlines.
 *     The function returns the old pointer.
 */
Fortify_OutputFuncPtr FORTIFY_STORAGE
_Fortify_SetOutputFunc(Fortify_OutputFuncPtr Output)
{
	OutputFuncPtr Old = st_Output;

	st_Output = (OutputFuncPtr) Output;

	return((Fortify_OutputFuncPtr FORTIFY_STORAGE) Old);
}

/*
 * _Fortify_SetMallocFailRate(int Percent) - _Fortify_malloc() will make the
 *   malloc attempt fail this Percent of the time, even if the memory is
 *   available. Useful to "stress-test" an application. Returns the old
 *   value. The fail rate defaults to 0.
 */
int FORTIFY_STORAGE
_Fortify_SetMallocFailRate(int Percent)
{
	int Old = st_MallocFailRate;

	st_MallocFailRate = Percent;

	return(Old);
}


/*
 * _Fortify_CheckAllMemory() - Checks the sentinals of all malloc'd memory.
 *   Returns the number of blocks that failed.
 *
 *  (If Fortify is disabled, this function always returns 0).
 */
int FORTIFY_STORAGE
_Fortify_CheckAllMemory(char *file,unsigned long line)
{
	struct Header *curr = st_Head;
	int count = 0;

	stdOutput = 0;

	if(st_Disabled)
	{
		WaitIfstdOutput();
		return(0);
	}

	FORTIFY_LOCK();

	while(curr)
	{
		if(!CheckBlock(curr, file, line))
			count++;

		curr = curr->Next;
	}

	if(file)
	{
		st_LastVerifiedFile = file;
		st_LastVerifiedLine = (short) line;
	}

	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(count);
}

/* _Fortify_EnterScope - enters a new Fortify scope level.
 * returns the new scope level.
 */
int FORTIFY_STORAGE
_Fortify_EnterScope(char *file,unsigned long line)
{
	return((int) ++st_Scope);
}

/* _Fortify_LeaveScope - leaves a Fortify scope level,
 * also prints a memory dump of all non-freed memory that was allocated
 * during the scope being exited.
 */
int FORTIFY_STORAGE
_Fortify_LeaveScope(char *file,unsigned long line)
{
	struct Header *curr = st_Head;
	int count = 0;
	size_t size = 0;

        stdOutput = 0;

	if(st_Disabled)
        {
                WaitIfstdOutput();
                return(0);
        }

	FORTIFY_LOCK();

	st_Scope--;
	while(curr)
	{
		if(curr->Scope > st_Scope)
		{
			if(count == 0)
			{
				sprintf(st_Buffer, "\nFortify: Memory Dump at %s.%ld\n", file, line);
				st_Output(st_Buffer);
				OutputLastVerifiedPoint();
				sprintf(st_Buffer, "%11s %8s %s\n", "Address", "Size", "Allocator");
				st_Output(st_Buffer);
			}

			OutputHeader(curr);
			count++;
			size += curr->Size;
		}

		curr = curr->Next;
	}

	if(count)
	{
		sprintf(st_Buffer, "%11s %8ld bytes overhead\n", "and",
						(unsigned long)(count * (sizeof(struct Header) + FORTIFY_BEFORE_SIZE + FORTIFY_AFTER_SIZE)));
						st_Output(st_Buffer);

		sprintf(st_Buffer,"%11s %8ld bytes in %d blocks\n", "total", size, count);
		st_Output(st_Buffer);
	}

	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(count);
}

/*
 * _Fortify_OutputAllMemory() - Outputs the entire list of currently
 *     malloc'd memory. For each malloc'd block is output it's Address,
 *   Size, and the SourceFile and Line that allocated it.
 *
 *   If there is no memory on the list, this function outputs nothing.
 *
 *   It returns the number of blocks on the list, unless fortify has been
 *   disabled, in which case it always returns 0.
 */
int FORTIFY_STORAGE
_Fortify_OutputAllMemory(char *file,unsigned long line)
{
	struct Header *curr = st_Head;
	int count = 0;
	size_t size = 0;

        stdOutput = 0;

	if(st_Disabled)
        {
                WaitIfstdOutput();
                return(0);
        }

	FORTIFY_LOCK();

	if(curr)
	{
		sprintf(st_Buffer, "\nFortify: Memory Dump at %s.%ld\n", file, line);
		st_Output(st_Buffer);
		OutputLastVerifiedPoint();
		sprintf(st_Buffer, "%11s %8s %s\n", "Address", "Size", "Allocator");
		st_Output(st_Buffer);

		while(curr)
		{
			OutputHeader(curr);
			count++;
			size += curr->Size;
			curr = curr->Next;
		}

		sprintf(st_Buffer, "%11s %8ld bytes overhead\n", "and",
				(unsigned long)(count * (sizeof(struct Header) + FORTIFY_BEFORE_SIZE + FORTIFY_AFTER_SIZE)));
		st_Output(st_Buffer);

		sprintf(st_Buffer,"%11s %8ld bytes in %d blocks\n", "total", size, count);
		st_Output(st_Buffer);
	}

	FORTIFY_UNLOCK();
        WaitIfstdOutput();
	return(count);
}

/* _Fortify_DumpAllMemory(Scope) - Outputs the entire list of currently
 * new'd memory within the specified scope. For each new'd block is output
 * it's Address, Size, the SourceFile and Line that allocated it, a hex dump
 * of the contents of the memory and an ascii dump of printable characters.
 *
 * If there is no memory on the list, this function outputs nothing.
 *
 * It returns the number of blocks on the list, unless Fortify has been
 * disabled, in which case it always returns 0.
 */
int FORTIFY_STORAGE
_Fortify_DumpAllMemory(int scope,char *file,unsigned long line)
{
	struct Header *curr = st_Head;
	int count = 0;
	size_t size = 0;

        stdOutput = 0;

	if(st_Disabled)
        {
                WaitIfstdOutput();
		return(0);
        }

	FORTIFY_LOCK();

	while(curr)
	{
		if(curr->Scope >= scope)
		{
			if(count == 0)
			{
				sprintf(st_Buffer, "\nFortify: Memory Dump at %s.%ld\n", file, line);
				st_Output(st_Buffer);
				OutputLastVerifiedPoint();
				sprintf(st_Buffer, "%11s %8s %s\n", "Address", "Size", "Allocator");
				st_Output(st_Buffer);
			}

			OutputHeader(curr);
			OutputMemory(curr);
			st_Output("\n");
			count++;
			size += curr->Size;
		}

		curr = curr->Next;
	}

	if(count)
	{
		sprintf(st_Buffer, "%11s %8ld bytes overhead\n", "and",
						(unsigned long)(count * (sizeof(struct Header) + FORTIFY_BEFORE_SIZE + FORTIFY_AFTER_SIZE)));
						st_Output(st_Buffer);

		sprintf(st_Buffer,"%11s %8ld bytes in %d blocks\n", "total", size, count);
		st_Output(st_Buffer);
	}

	FORTIFY_UNLOCK();
        WaitIfstdOutput();
        return(count);
}

/*
 * _Fortify_Disable() - This function provides a mechanism to disable Fortify
 *   without recompiling all the sourcecode.
 *   If 'how' is zero then it can only be called when there is no memory on the
 *   Fortify malloc'd list. (Ideally, at the start of the program before any
 *   memory has been allocated). If you call this function when there IS
 *   memory on the Fortify malloc'd list, it will issue an error, and fortify
 *   will not be disabled.
 *   If 'how' is nonzero then output will only be disabled. This can always be
 *   done.
 */

int FORTIFY_STORAGE
_Fortify_Disable(char *file,unsigned long line,int how)
{
	int result;

        if (how == 0)
        {
    	    stdOutput = 0;

    	    FORTIFY_LOCK();

    	    if(st_Head)
    	    {
    		    sprintf(st_Buffer, "Fortify: %s.%d\n", file, line);
    		    st_Output(st_Buffer);
    		    st_Output("         Fortify_Disable failed\n");
    		    st_Output("         (because there is memory on the Fortify memory list)\n");

    		    _Fortify_OutputAllMemory(file, line);
    		    result = 0;
    	    }
    	    else
    	    {
    		    st_Disabled = 1;
    		    result = 1;
    	    }

    	    FORTIFY_UNLOCK();
            WaitIfstdOutput();
        }
        else
        {
            _Fortify_SetOutputFunc((Fortify_OutputFuncPtr) _Fortify_NoOutput);
            result = 1;
        }
	return(result);
}

/*
 * Check a block's header and fortifications.
 */
static int CheckBlock(struct Header *h,char *file,unsigned long line)
{
	unsigned char *ptr = (unsigned char *)h;
	int result = 1;

        stdOutput = 0;

	if(!IsHeaderValid(h))
	{
		sprintf(st_Buffer,
				"\nFortify: %s.%ld\n         Invalid pointer or corrupted header detected (%s)\n",
				file, line, address(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE));
		st_Output(st_Buffer);
		OutputLastVerifiedPoint();
                WaitIfstdOutput();
                return(0);
	}

	if(!CheckFortification(ptr + sizeof(struct Header),
		                   FORTIFY_BEFORE_VALUE, FORTIFY_BEFORE_SIZE))
	{
		sprintf(st_Buffer,
		        "\nFortify: %s.%ld\n         Memory overrun detected before block\n",
		        file, line);
		st_Output(st_Buffer);

		sprintf(st_Buffer,"         (%s,%ld,%s.%u)\n",
			address(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE),
		       (unsigned long)h->Size, h->File, h->Line);
		st_Output(st_Buffer);

		OutputFortification(ptr + sizeof(struct Header),
		                    FORTIFY_BEFORE_VALUE, FORTIFY_BEFORE_SIZE);
		OutputLastVerifiedPoint();
		result = 0;
	}

	if(!CheckFortification(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE + h->Size,
	                       FORTIFY_AFTER_VALUE, FORTIFY_AFTER_SIZE))
	{
		sprintf(st_Buffer, "\nFortify: %s.%ld\n         Memory overrun detected after block\n",
		                   file, line);
		st_Output(st_Buffer);

		sprintf(st_Buffer,"         (%s,%ld,%s.%u)\n",
				  address(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE),
				 (unsigned long)h->Size, h->File, h->Line);
		st_Output(st_Buffer);

		OutputFortification(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE + h->Size,
		                    FORTIFY_AFTER_VALUE, FORTIFY_AFTER_SIZE);
		OutputLastVerifiedPoint();
		result = 0;
	}

        WaitIfstdOutput();
	return(result);
}

static int CheckPointer(unsigned char *ptr,unsigned long size,char *file,unsigned long line)
{
	struct Header *curr;
	unsigned char *ptr1;

	curr = st_Head;
	while(curr)
	{
		ptr1 = (unsigned char *)curr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE;
		if(ptr + size <= (unsigned char *)curr)
			;
		else if(ptr >= ptr1)
			;
		else
		{
			sprintf(st_Buffer, "\nFortify: %s.%ld\n         Memory access detected before block\n",
		                file, line);
			st_Output(st_Buffer);

			sprintf(st_Buffer,"         (%s,%ld,%s.%u)\n",
					address(ptr1),
					(unsigned long)curr->Size, curr->File, curr->Line);
			st_Output(st_Buffer);

			WaitIfstdOutput();
			return(0);
		}

		if(ptr + size <= ptr1 + curr->Size)
			;
		else if(ptr >= ptr1 + curr->Size + FORTIFY_AFTER_SIZE)
			;
		else
		{
			sprintf(st_Buffer, "\nFortify: %s.%ld\n         Memory access detected after block\n",
		                file, line);
			st_Output(st_Buffer);

			sprintf(st_Buffer,"         (%s,%ld,%s.%u)\n",
					address(ptr1),
					(unsigned long)curr->Size, curr->File, curr->Line);
			st_Output(st_Buffer);

			WaitIfstdOutput();
			return(0);
		}

		if((ptr >= ptr1) && (ptr < ptr1 + curr->Size) && (ptr + size > ptr1 + curr->Size))
		{
			sprintf(st_Buffer, "\nFortify: %s.%ld\n         Memory access detected after block\n",
		                file, line);
			st_Output(st_Buffer);

			sprintf(st_Buffer,"         (%s,%ld,%s.%u)\n",
					address(ptr + sizeof(struct Header) + FORTIFY_BEFORE_SIZE),
					(unsigned long)curr->Size, curr->File, curr->Line);
			st_Output(st_Buffer);
			WaitIfstdOutput();
			return(0);
		}

		curr = curr->Next;
	}
	return(1);
}

/*
 * Checks if the _size_ bytes from _ptr_ are all set to _value_
 */
static int CheckFortification(unsigned char *ptr,unsigned char value,size_t size)
{
	while(size--)
		if(*ptr++ != value)
			return(0);

	return(1);
}

/*
 * Set the _size_ bytes from _ptr_ to _value_.
 */
static void SetFortification(unsigned char *ptr,unsigned char value,size_t size)
{
	memset(ptr, value, size);
}

/*
 * Output the corrupted section of the fortification
 */
/* Output the corrupted section of the fortification */
static void
OutputFortification(unsigned char *ptr,unsigned char value,size_t size)
{
	unsigned long offset, column;
	char	ascii[17];

	st_Output("Address     Offset Data");

	offset = 0;
	column = 0;

	while(offset < size)
	{
		if(column == 0)
		{
			sprintf(st_Buffer, "\n%8s %8d ", address(ptr), offset);
			st_Output(st_Buffer);
		}

		sprintf(st_Buffer, "%02x ", *ptr);
		st_Output(st_Buffer);

		ascii[ (int) column ] = isprint( *ptr ) ? (char)(*ptr) : (char)(' ');
		ascii[ (int) (column + 1) ] = '\0';

		ptr++;
		offset++;
		column++;

		if(column == 16)
		{
			st_Output( "   \"" );
			st_Output( ascii );
			st_Output( "\"" );
			column = 0;
		}
	}

	if ( column != 0 )
	{
		while ( column ++ < 16 )
		{
			st_Output( "   " );
		}
		st_Output( "   \"" );
		st_Output( ascii );
		st_Output( "\"" );
	}

	st_Output("\n");
}

/*
 * Returns true if the supplied pointer does indeed point to a real Header
 */
static int IsHeaderValid(struct Header *h)
{
	return(!ChecksumHeader(h));
}

/*
 * Updates the checksum to make the header valid
 */
static void MakeHeaderValid(struct Header *h)
{
	h->Checksum = 0;
	h->Checksum = -ChecksumHeader(h);
}

/*
 * Calculate (and return) the checksum of the header. (Including the Checksum
 * variable itself. If all is well, the checksum returned by this function should
 * be 0.
 */
static int ChecksumHeader(struct Header *h)
{
	register int c, checksum, *p;

	for(c = 0, checksum = 0, p = (int *)h; c < sizeof(struct Header)/sizeof(int); c++)
		checksum += *p++;

	return(checksum);
}

/*
 * Examines the malloc'd list to see if the given header is on it.
 */
static int IsOnList(struct Header *h)
{
	struct Header *curr;

	curr = st_Head;
	while(curr)
	{
		if(curr == h)
			return(1);
		curr = curr->Next;
	}

	return(0);
}


/*
 * Hex and ascii dump the memory
 */
static void
OutputMemory(struct Header *h)
{
	OutputFortification((unsigned char*)h + sizeof(struct Header) + FORTIFY_BEFORE_SIZE,
						0, h->Size);
}


/*
 * Output the header...
 */
static void OutputHeader(struct Header *h)
{
	sprintf(st_Buffer, "%11s %8ld %s.%u (%d)\n",
			   address((unsigned char*)h + sizeof(struct Header) + FORTIFY_BEFORE_SIZE),
			   (unsigned long)h->Size,
			   h->File, h->Line, (int) h->Scope);
	st_Output(st_Buffer);
}

static void OutputLastVerifiedPoint()
{
	sprintf(st_Buffer, "\nLast Verified point: %s.%u\n",
			   st_LastVerifiedFile,
			   st_LastVerifiedLine);
	st_Output(st_Buffer);
}

#else  /* FORTIFY_TRANSPARENT */

void *FORTIFY_STORAGE
_Fortify_malloc(size,file,line)
 size_t size;
 char *file;
 unsigned long line;
{
	return(malloc(size));
}

void FORTIFY_STORAGE
_Fortify_free(uptr,file,line)
 void *uptr;
 char *file;
 unsigned long line;
{
	free(uptr);
}

void *FORTIFY_STORAGE
_Fortify_realloc(ptr,new_size,file,line)
 void *ptr;
 size_t new_size;
 char *file;
 unsigned long line;
{
	return(realloc(ptr, new_size));
}

int FORTIFY_STORAGE
_Fortify_CheckPointer(uptr,file,line)
 void *uptr;
 char *file;
 unsigned long line;
{
	return(1);
}

Fortify_OutputFuncPtr FORTIFY_STORAGE
_Fortify_SetOutputFunc(Output)
 Fortify_OutputFuncPtr Output;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_SetMallocFailRate(Percent)
 int Percent;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_CheckAllMemory(file,line)
 char *file;
 unsigned long line;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_EnterScope(file,line)
 char *file;
 unsigned long line;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_LeaveScope(file,line)
 char *file;
 unsigned long line;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_OutputAllMemory(file,line)
 char *file;
 unsigned long line;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_DumpAllMemory(scope,file,line)
 int scope;
 char *file;
 unsigned long line;
{
	return(0);
}

int FORTIFY_STORAGE
_Fortify_Disable(file,line)
 char *file;
 unsigned long line;
{
	return(1);
}

#endif /* !FORTIFY_TRANSPARENT */

/* function that use _Fortify_malloc(), _Fortify_realloc(), _Fortify_free() */

/*
 * Fortifty_calloc() - Uses _Fortify_malloc() to implement calloc(). Much
 *                     the same protection as _Fortify_malloc().
 */
void *FORTIFY_STORAGE
_Fortify_calloc(size_t nitems,size_t size,char *file,unsigned long line)
{
	void *ptr;

	ptr = _Fortify_malloc(nitems * size, file, line);

	if(ptr)
		memset(ptr, 0, nitems * size);

	return(ptr);
}

/*
 * Fortifty_strdup() - Uses _Fortify_malloc() to implement strdup(). Much
 *                     the same protection as _Fortify_malloc().
 * The library function is not used because it is not certain that getpwd
 * uses the library malloc function (if linked with an alternate library)
 * and if the memory is freed then strange things can happen
 */
char *FORTIFY_STORAGE
_Fortify_strdup(char *str,char *file,unsigned long line)
{
	char *ptr;
	unsigned long l;

	if(str == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "strdup pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	l = strlen(str) + 1;
	__Fortify_CheckPointer(str,0,l,file,line);

	ptr = (char *) _Fortify_malloc(l, file, line);

	if(ptr)
		strcpy(ptr, str);

	return(ptr);
}

/*
 * Fortifty_getpwd() - Uses _Fortify_malloc() to implement getpwd(). Much
 *                     the same protection as _Fortify_malloc().
 * Memory is not allocated bu getcwd but by our routine for the same reason
 * as for strdup
 */
char *FORTIFY_STORAGE
_Fortify_getcwd(char *buf,int size,char *file,unsigned long line)
{
	char *ptr;

        if(buf!=NULL)
                ptr = buf;
	else
        	ptr = (char *) _Fortify_malloc(size + 1, file, line);

	if(ptr)
		ptr = getcwd(ptr, size);

	return(ptr);
}

/*
 * Fortifty_tempnam() - Uses _Fortify_strdup() to implement tempnam(). Much
 *                     the same protection as _Fortify_malloc().
 */
char *FORTIFY_STORAGE
_Fortify_tempnam(char *dir,char *pfx,char *file,unsigned long line)
{
	char *ptr1, *ptr2;

        ptr1 = tempnam(dir,pfx);

	if(ptr1)
        {
                ptr2=_Fortify_strdup(ptr1,file,line);
                free(ptr1);
                ptr1=ptr2;
        }

	return(ptr1);
}

/*
 * Fortify_memcpy()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
void *FORTIFY_STORAGE
_Fortify_memcpy(void *to,void *from,size_t size,char *file,unsigned long line)
{
	if((from == NULL) || (to == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(from == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "memcpy from pointer is NULL", file, line);
		if(to == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (from == NULL) ? "" : " and ", "memcpy to pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(to,0,size,file,line);
	__Fortify_CheckPointer(from,0,size,file,line);
	return(memcpy(to,from,size));
}

/*
 * Fortify_memmove()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
void *FORTIFY_STORAGE
_Fortify_memmove(void *to,void *from,size_t size,char *file,unsigned long line)
{
	if((from == NULL) || (to == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(from == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "memmove from pointer is NULL", file, line);
		if(to == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (from == NULL) ? "" : " and ", "memmove to pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(to,0,size,file,line);
	__Fortify_CheckPointer(from,0,size,file,line);
	return(memmove(to,from,size));
}

/*
 * Fortify_memccpy()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
void *FORTIFY_STORAGE
_Fortify_memccpy(void *to,void *from,int c,size_t size,char *file,unsigned long line)
{
	if((from == NULL) || (to == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(from == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "memccpy from pointer is NULL", file, line);
		if(to == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (from == NULL) ? "" : " and ", "memccpy to pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(to,0,size,file,line);
	__Fortify_CheckPointer(from,0,size,file,line);
	return(memccpy(to,from,c,size));
}

/*
 * Fortify_memset()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
void *FORTIFY_STORAGE
_Fortify_memset(void *buffer,int c,size_t size,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         memset pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,size,file,line);
	return(memset(buffer,c,size));
}

/*
 * Fortify_memchr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
void *FORTIFY_STORAGE
_Fortify_memchr(void *buffer,int c,size_t size,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         memchr pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,size,file,line);
	return(memchr(buffer,c,size));
}

/*
 * Fortify_memcmp()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_memcmp(void *buffer1,void *buffer2,size_t size,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "memcmp first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "memcmp second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,size,file,line);
	__Fortify_CheckPointer(buffer2,0,size,file,line);
	return(memcmp(buffer1,buffer2,size));
}

/*
 * Fortify_memicmp()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_memicmp(void *buffer1,void *buffer2,size_t size,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "memicmp first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "memicmp second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,size,file,line);
	__Fortify_CheckPointer(buffer2,0,size,file,line);
	return(memicmp(buffer1,buffer2,size));
}

/*
 * Fortify_strcoll()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_strcoll(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strcoll first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strcoll second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,strlen(buffer2)+1,file,line);
	return(strcoll(buffer1,buffer2));
}

/*
 * Fortify_strcspn()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
size_t FORTIFY_STORAGE
_Fortify_strcspn(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strcspn first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strcspn second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,strlen(buffer2)+1,file,line);
	return(strcspn(buffer1,buffer2));
}

/*
 * Fortify_strcmp()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_strcmp(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strcmp first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strcmp second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,strlen(buffer2)+1,file,line);
	return(strcmp(buffer1,buffer2));
}

/*
 * Fortify_strcmpi()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_strcmpi(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strcmpi first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strcmpi second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,strlen(buffer2)+1,file,line);
	return(strcmpi(buffer1,buffer2));
}

/*
 * Fortify_strncmp()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_strncmp(char *buffer1,char *buffer2,size_t size,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strncmp first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strncmp second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,MIN(strlen(buffer1)+1,size),file,line);
	__Fortify_CheckPointer(buffer2,0,MIN(strlen(buffer2)+1,size),file,line);
	return(strncmp(buffer1,buffer2,size));
}

/*
 * Fortify_strnicmp()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_strnicmp(char *buffer1,char *buffer2,size_t size,char *file,unsigned long line)
{
	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strnicmp first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strnicmp second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,MIN(strlen(buffer1)+1,size),file,line);
	__Fortify_CheckPointer(buffer2,0,MIN(strlen(buffer2)+1,size),file,line);
	return(strnicmp(buffer1,buffer2,size));
}

/*
 * Fortify_strchr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strchr(char *buffer,int c,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strchr pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strchr(buffer,c));
}

/*
 * Fortify_strrchr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strrchr(char *buffer,int c,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strchr pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strrchr(buffer,c));
}

/*
 * Fortify_strlwr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strlwr(char *buffer,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strlwr pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strlwr(buffer));
}

/*
 * Fortify_strlwr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strupr(char *buffer,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strupr pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strupr(buffer));
}

/*
 * Fortify_strrev()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strrev(char *buffer,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strrev pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strrev(buffer));
}

/*
 * Fortify_strlen()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
size_t FORTIFY_STORAGE
_Fortify_strlen(char *buffer,char *file,unsigned long line)
{
	unsigned long l;

	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strlen pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	l = strlen(buffer);
	__Fortify_CheckPointer(buffer,0,l+1,file,line);
	return(l);
}

/*
 * Fortify_strcat()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strcat(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	unsigned long l;

	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strcat first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strcat second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	l = strlen(buffer2)+1;
	__Fortify_CheckPointer(buffer1,0,l,file,line);
	__Fortify_CheckPointer(buffer2,0,l,file,line);
	return(strcat(buffer1,buffer2));
}

/*
 * Fortify_strpbrk()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strpbrk(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	unsigned long l;

	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strpbrk first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strpbrk second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	l = strlen(buffer2)+1;
	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,l,file,line);
	return(strpbrk(buffer1,buffer2));
}

/*
 * Fortify_strstr()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strstr(char *buffer1,char *buffer2,char *file,unsigned long line)
{
	unsigned long l;

	if((buffer1 == NULL) || (buffer2 == NULL)) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		if(buffer1 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s", "strstr first pointer is NULL", file, line);
		if(buffer2 == NULL)
			sprintf(st_Buffer + strlen(st_Buffer), "%s%s", (buffer2 == NULL) ? "" : " and ", "strstr second pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	l = strlen(buffer2)+1;
	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	__Fortify_CheckPointer(buffer2,0,l,file,line);
	return(strstr(buffer1,buffer2));
}

/*
 * Fortify_strtol()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
long FORTIFY_STORAGE
_Fortify_strtol(char *buffer1,char **buffer2,int n,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "strtol first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(strtol(buffer1,buffer2,n));
}

/*
 * Fortify_atoi()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
int FORTIFY_STORAGE
_Fortify_atoi(char *buffer1,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "atoi first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(atoi(buffer1));
}

/*
 * Fortify_atol()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
long FORTIFY_STORAGE
_Fortify_atol(char *buffer1,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "atol first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(atol(buffer1));
}

/*
 * Fortify_atod()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
double FORTIFY_STORAGE
_Fortify_atof(char *buffer1,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "atod first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(atof(buffer1));
}

/*
 * Fortify_strtoul()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
unsigned long FORTIFY_STORAGE
_Fortify_strtoul(char *buffer1,char **buffer2,int n,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "strtoul first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(strtoul(buffer1,buffer2,n));
}

/*
 * Fortify_strtod()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
double FORTIFY_STORAGE
_Fortify_strtod(char *buffer1,char **buffer2,char *file,unsigned long line)
{
	if(buffer1 == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         ", file, line);
		sprintf(st_Buffer + strlen(st_Buffer), "%s", "strtod first pointer is NULL", file, line);
		strcat(st_Buffer, "\n");
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(0);
	}

	__Fortify_CheckPointer(buffer1,0,strlen(buffer1)+1,file,line);
	return(strtod(buffer1,buffer2));
}

/*
 * Fortify_strset()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strset(char *buffer,int c,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strset pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strset(buffer,c));
}

/*
 * Fortify_strnset()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strnset(char *buffer,int c,size_t size,char *file,unsigned long line)
{
	if(buffer == NULL) {
		sprintf(st_Buffer,
			"\nFortify: %s.%ld\n         strnset pointer is NULL\n", file, line);
		st_Output(st_Buffer);
		FORTIFY_UNLOCK();
		WaitIfstdOutput();
		return(NULL);
	}

	__Fortify_CheckPointer(buffer,0,strlen(buffer)+1,file,line);
	return(strnset(buffer,c,size));
}

/*
 * Fortify_strncpy()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
static char *FORTIFY_STORAGE
__Fortify_strncpy(char *to,char *from,size_t size,int usesize,char *file,unsigned long line)
{
	size_t size1;

	size1 = strlen(from) + 1;
	if(usesize)
	{
		if(size < size1)
			size1 = size;
	}

	return((char *) _Fortify_memcpy(to,from,size1,file,line));
}

/*
 * Fortify_strncpy()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strncpy(char *to,char *from,size_t size,char *file,unsigned long line)
{
	return(__Fortify_strncpy(to,from,size,1,file,line));
}

/*
 * Fortify_strncpy()  - check if from/to is in allocated space and if so, then check if start/end is not outside allocated space.
 */
char *FORTIFY_STORAGE
_Fortify_strcpy(char *to,char *from,char *file,unsigned long line)
{
	return(__Fortify_strncpy(to,from,0,0,file,line));
}

#endif /* FORTIFY */
