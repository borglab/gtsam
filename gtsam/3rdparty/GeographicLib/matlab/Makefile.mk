MATLAB_FILES = $(wildcard geographiclib/*.m)
MATLAB_PRIVATE = $(wildcard geographiclib/private/*.m)
MATLAB_LEGACY = $(wildcard geographiclib-legacy/*.m)

DEST = $(PREFIX)/share/matlab
INSTALL = install -b

all:
	@:

install:
	test -d $(DEST)/geographiclib/private || \
		mkdir -p $(DEST)/geographiclib/private
	test -d $(DEST)/geographiclib-legacy || \
		mkdir -p $(DEST)/geographiclib-legacy
	$(INSTALL) -m 644 $(MATLAB_FILES) $(DEST)/geographiclib
	$(INSTALL) -m 644 $(MATLAB_PRIVATE) $(DEST)/geographiclib/private/
	$(INSTALL) -m 644 $(MATLAB_LEGACY) $(DEST)/geographiclib-legacy

clean:
	rm -f *.mex* *.oct

.PHONY: all install clean
