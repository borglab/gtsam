#-----------------------------------------------------------------------------
# compile the CCOLAMD demo
#-----------------------------------------------------------------------------

default: all

include ../../SuiteSparse_config/SuiteSparse_config.mk

I = -I../../include

C = $(CC) $(CF) $(I)

LIB2 = $(LDFLAGS) -L../../lib -lccolamd -lsuitesparseconfig $(LDLIBS)

all: library ccolamd_example ccolamd_l_example

library:
	( cd ../../SuiteSparse_config ; $(MAKE) )
	( cd ../Lib ; $(MAKE) )

#------------------------------------------------------------------------------
# Create the demo program, run it, and compare the output
#------------------------------------------------------------------------------

dist:

ccolamd_example: ccolamd_example.c
	$(C) -o ccolamd_example ccolamd_example.c $(LIB2)
	- ./ccolamd_example > my_ccolamd_example.out
	- diff ccolamd_example.out my_ccolamd_example.out

ccolamd_l_example: ccolamd_l_example.c
	$(C) -o ccolamd_l_example ccolamd_l_example.c $(LIB2)
	- ./ccolamd_l_example > my_ccolamd_l_example.out
	- diff ccolamd_l_example.out my_ccolamd_l_example.out

#------------------------------------------------------------------------------
# Remove all but the files in the original distribution
#------------------------------------------------------------------------------

clean:
	- $(RM) -r $(CLEAN)

purge: distclean

distclean: clean
	- $(RM) ccolamd_example ccolamd_l_example
	- $(RM) my_ccolamd_example.out my_ccolamd_l_example.out
	- $(RM) -r $(PURGE)
