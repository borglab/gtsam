MAKEFILE := $(lastword $(MAKEFILE_LIST))
MAKE := $(MAKE) -f $(MAKEFILE)
SUBDIRS = src man tools doc js
ALLDIRS = include $(SUBDIRS) maxima matlab python cmake

all: src man tools js

$(SUBDIRS):
	$(MAKE) -C $@

tools: src
install: install-headers install-lib install-tools install-man install-cmake \
	install-doc install-js install-matlab install-python
clean: clean-src clean-tools clean-doc clean-js clean-man clean-matlab \
	clean-python

install-headers:
	$(MAKE) -C include install
install-lib:
	$(MAKE) -C src install
install-tools: src
	$(MAKE) -C tools install
install-cmake:
	$(MAKE) -C cmake install
install-doc: doc
	$(MAKE) -C doc install
install-js: js
	$(MAKE) -C js install
install-man: man
	$(MAKE) -C man install
install-matlab: matlab
	$(MAKE) -C matlab install
install-python: python
	$(MAKE) -C python install
clean-src:
	$(MAKE) -C src clean
clean-tools:
	$(MAKE) -C tools clean
clean-doc:
	$(MAKE) -C doc clean
clean-js:
	$(MAKE) -C js clean
clean-man:
	$(MAKE) -C man clean
clean-matlab: matlab
	$(MAKE) -C matlab clean
clean-python: python
	$(MAKE) -C python clean

VERSION:=$(shell grep '\bVERSION=' configure | cut -f2 -d\' | head -1)

.PHONY: all $(SUBDIRS) install clean \
	install-headers install-lib install-tools install-man install-cmake \
	install-doc install-js install-matlab install-python \
	clean-src clean-tools clean-doc clean-js clean-man clean-matlab \
	clean-python
