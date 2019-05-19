MODULES = __init__ geomath constants accumulator geodesiccapability \
	geodesic geodesicline polygonarea
PACKAGE = geographiclib
PYTHON_FILES = $(patsubst %,$(PACKAGE)/%.py,$(MODULES))
TEST_FILES = $(PACKAGE)/test/__init__.py $(PACKAGE)/test/test_geodesic.py

DEST = $(PREFIX)/lib/python/site-packages/$(PACKAGE)
INSTALL = install -b

all:
	@:

install:
	test -d $(DEST)/test || mkdir -p $(DEST)/test
	$(INSTALL) -m 644 $(PYTHON_FILES) $(DEST)/
	$(INSTALL) -m 644 $(TEST_FILES) $(DEST)/test/

# Don't install setup.py because it ends up in e.g.,
# /usr/local/lib/python/site-packages/setup.py
#	$(INSTALL) -m 644 setup.py $(DEST)/../

clean:
	rm -f *.pyc $(PACKAGE)/*.pyc

.PHONY: all install clean
