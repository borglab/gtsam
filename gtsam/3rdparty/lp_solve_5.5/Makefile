CC= gcc

MINGW_EXTRA = c:/cygwin/mingw
MINGW_INCDIR = $(MINGW_EXTRA)/include
MINGW_LIBDIR = $(MINGW_EXTRA)/lib

AS = as
DLLTOOL = dlltool
DLLWRAP = dllwrap
DLLWRAP_FLAGS = --dlltool-name=$(DLLTOOL) --as=$(AS) --driver-name=$(CC) \
		--target=i386-mingw32 

DLL_LDFLAGS = -L$(MINGW_LIBDIR) -mno-cygwin -mwindows
DLL_LDLIBS = liblpk.a

#
# Beginning of supplied stuff
#

#should be OK in most situations:
#CFLAGS= -O

# HP/UX 9.0X optimized code
#CFLAGS= +O3 +Oaggressive +Olibcalls -Aa -D_POSIX_SOURCE -DCHECK +FP VZOUiD
# HP/UX 9.0X debugging
#CFLAGS= -g -Aa -D_POSIX_SOURCE -DCHECK +FP VZOUiD

# nice for gcc
CFLAGS= -O3 -Wall -pedantic -ansi -mrtd
#CFLAGS= -g -Wall -pedantic -ansi
INCLUDE=-Ibfp -Ibfp/bfp_etaPFI -I. -Icolamd -Ishared
CFLAGS= -O3 -Wall -pedantic -ansi -mrtd -mno-cygwin -I$(MINGW_INCDIR)\
-DINTEGERTIME -DPARSER_LP -DYY_INTERACTIVE -DWIN32 $(INCLUDE) \
-DBUILDING_FOR_R -DWIN32 -g

# Option -DCHECK checks for numerical problems during rounding of numbers.
# It will slow things down a bit.
# You can add a -DREAL=<float type> to the CFLAGS, to change the default float
# type used in lp_solve (double) to float or 'long double'. However, type float
# might be fast on your computer, but it is not accurate enough to solve even
# moderately sized problems without running into numerical problems.
# The use of long doubles does increase the numerical stability of lp_solve,
# if your compiler actually implements them with more bits than a double. But
# it slows down things quite a bit.

# Choose your favorite or available version of lex and yacc

#YACC= yacc
#especially for linux:
YACC= bison -y

#LEX= lex
#especially for linux:
LEX= flex -l

#LEXLIB= -ll
#especially for linux:
LEXLIB= -lfl

#ANSI math lib
#MATHLIB= -lM
#non-ANSI math lib, should also work
MATHLIB= -lm

LPKSRC.c= $(wildcard *.c) lp_MDO.c \
colamd/colamd.c shared/commonlib.c shared/myblas.c

LEXFILE.l= lp_rlp.l
YACCFILE.y= lp_rlp.y

LPKLIB=liblpk.a

LEXFILE.c= $(LEXFILE.l:.l=.c)
YACCFILE.c= $(YACCFILE.y:.y=.c)
YACCFILE.o= $(YACCFILE.y:.y=.o)
CSOURCES=$(LEXFILE.c) $(YACCFILE.c) $(wildcard *.c)
# commonlib.c fortify.c lp_Hash.c \
# lp_LUMOD.c lp_MPS.c lp_SOS.c lp_crash.c lp_lib.c lp_matrix.c lp_mipbb.c \
# lp_presolve.c lp_price.c lp_pricePSE.c lp_report.c lp_rlp.c lp_scale.c \
# lp_simplex.c lp_solveDLL.c lp_utils.c lp_wlp.c lpslink.c myblas.c \
# yacc_read.c
COBJ=$(CSOURCES:.c=.o)
LPKSRC= $(LPKSRC.c) $(YACCFILE.c)
LPKOBJ= $(LPKSRC:.c=.o)
HEADERS=*.h

all:	lpslink.dll

q8.exe:	q8.o
	$(CC) -o q8.exe $(CFLAGS) \
--allow-multiple-definition \
q8.o colamd/colamd.o shared/commonlib.o \
fortify.o ini.o \
lp_Hash.o lp_LUSOL.o lp_MDO.o lp_MPS.o lp_SOS.o lp_crash.o lp_lib.o \
lp_matrix.o lp_mipbb.o lp_params.o lp_presolve.o lp_price.o lp_pricePSE.o \
lp_report.o lp_rlp.o lp_scale.o lp_simplex.o lp_utils.o lp_wlp.o lpslink.o \
lusol.o mmio.o shared/myblas.o yacc_read.o \
$(MATHLIB) $(LPKLIB) \
-Wl,--allow-multiple-definition


$(COBJ): $(HEADERS)

purify: lp_solve.o $(LPKLIB)
	purify $(CC) -o $(TARGET) $(CFLAGS) lp_solve.o $(LPKLIB) $(LEXLIB) $(MATHLIB)

quantify: lp_solve.o $(LPKLIB)
	quantify $(CC) -o $(TARGET) $(CFLAGS) lp_solve.o $(LPKLIB) $(LEXLIB) $(MATHLIB)

$(LPKLIB): $(LPKOBJ)
	ar rv $@ $(LPKOBJ)
	ranlib $(LPKLIB)

$(YACCFILE.o): $(LEXFILE.c)

$(LEXFILE.c): $(LEXFILE.l)
	$(LEX) $(LEXFILE.l)
	mv lex.yy.c $(LEXFILE.c)

$(YACCFILE.c): $(YACCFILE.y)
	$(YACC) $(YACCFILE.y)
	mv y.tab.c $(YACCFILE.c)

# liblpk.def: lpslink.o
# 	dlltool -z liblpk.def -e exports.o -l liblpk.lib lpslink.o
lpslink.dll: lpslink.o liblpk.def liblpk.a
	$(DLLWRAP) $(DLLWRAP_FLAGS) -o $@ --def liblpk.def \
lpslink.o $(DLL_LDFLAGS) $(DLL_LDLIBS) 



test:
	-for i in $(TESTFILES1); do\
		./$(TARGET) -s -S3 -time < $$i > xxx.tmp;\
		if diff xxx.tmp lp_examples/`basename $$i .lp`.out > /dev/null; then\
			echo "$$i gives the correct result";\
		else\
			echo "*** $$i gives different result, please check ***";\
		fi;\
	done;\
	for i in $(TESTFILES2); do\
		./$(TARGET) -mps -s -S3 -time < $$i > xxx.tmp;\
		if diff xxx.tmp lp_examples/`basename $$i .mps`.out > /dev/null; then\
			echo "$$i gives the correct result";\
		else\
			echo "*** $$i gives different result, please check ***";\
		fi;\
	done;\
        rm xxx.tmp

mktest:
	-for i in $(TESTFILES1); do\
		./$(TARGET) -s -S3 -time < $$i > lp_examples/`basename $$i .lp`.out;\
	done;\
	for i in $(TESTFILES2); do\
		./$(TARGET) -mps -s -S3 -time < $$i > lp_examples/`basename $$i .mps`.out;\
	done;\

$(TARGET).man: $(TARGET).1
	nroff -man $(TARGET).1 > $(TARGET).man

MANIFEST: clean
	ls -lR > MANIFEST; ls -lR > MANIFEST

clean:
	rm -f *.a *.o TAGS $(LEXFILE.c) $(YACCFILE.c) demo $(TARGET) lp2mps mps2lp .pure .softdebughist datafile

TAGS:
	etags *.[chyl]
