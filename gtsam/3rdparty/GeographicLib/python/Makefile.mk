MODULES = __init__ geomath constants accumulator geodesiccapability \
	geodesic geodesicline polygonarea
PACKAGE = geographiclib
PYTHON_FILES = $(patsubst %,$(PACKAGE)/%.py,$(MODULES))

DEST = $(PREFIX)/lib/python/site-packages/$(PACKAGE)
INSTALL = install -b

all:
	@:

install:
	test -d $(DEST) || mkdir -p $(DEST)
	$(INSTALL) -m 644 $(PYTHON_FILES) $(DEST)/

# Don't install setup.py because it ends up in e.g.,
# /usr/local/lib/python/site-packages/setup.py
#	$(INSTALL) -m 644 setup.py $(DEST)/../

clean:
	rm -f *.pyc $(PACKAGE)/*.pyc

.PHONY: all install clean
