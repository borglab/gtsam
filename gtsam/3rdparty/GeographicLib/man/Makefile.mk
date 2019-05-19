PROGRAMS = CartConvert \
	ConicProj \
	GeoConvert \
	GeodSolve \
	GeodesicProj \
	GeoidEval \
	Gravity \
	MagneticField \
	Planimeter \
	RhumbSolve \
	TransverseMercatorProj

MANPAGES = $(addsuffix .1,$(PROGRAMS))
USAGE = $(addsuffix .usage,$(PROGRAMS))
HTMLMAN = $(addsuffix .1.html,$(PROGRAMS))

PREFIX = /usr/local

DEST = $(PREFIX)/share/man/man1

all: $(MANPAGES) $(USAGE) $(HTMLMAN)

INSTALL = install -b

install:
	test -d $(DEST) || mkdir -p $(DEST)
	$(INSTALL) -m 644 $(MANPAGES) $(DEST)/

.PHONY: all install clean
