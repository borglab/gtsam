#
# Makefile.am
#
# Copyright (C) 2011-2022, Charles Karney <karney@alum.mit.edu>

USAGE = \
	CartConvert.usage \
	ConicProj.usage \
	GeodesicProj.usage \
	GeoConvert.usage \
	GeodSolve.usage \
	GeoidEval.usage \
	Gravity.usage \
	IntersectTool.usage \
	MagneticField.usage \
	Planimeter.usage \
	RhumbSolve.usage \
	TransverseMercatorProj.usage

MANPAGES = \
	CartConvert.1 \
	ConicProj.1 \
	GeodesicProj.1 \
	GeoConvert.1 \
	GeodSolve.1 \
	GeoidEval.1 \
	Gravity.1 \
	IntersectTool.1 \
	MagneticField.1 \
	Planimeter.1 \
	RhumbSolve.1 \
	TransverseMercatorProj.1

HTMLMAN = \
	CartConvert.1.html \
	ConicProj.1.html \
	GeodesicProj.1.html \
	GeoConvert.1.html \
	GeodSolve.1.html \
	GeoidEval.1.html \
	Gravity.1.html \
	IntersectTool.1.html \
	MagneticField.1.html \
	Planimeter.1.html \
	RhumbSolve.1.html \
	TransverseMercatorProj.1.html

SYSMANPAGES = geographiclib-get-geoids.8 \
	geographiclib-get-gravity.8 \
	geographiclib-get-magnetic.8

man1_MANS = $(MANPAGES)
man8_MANS = $(SYSMANPAGES)

SUFFIXES = .1 .usage .1.html .8

all: man
man: manpages usage htmlman sysmanpages

manpages: $(MANPAGES)
usage: $(USAGE)
htmlman: $(HTMLMAN)
sysmanpages: $(SYSMANPAGES)

geographiclib_data = $(datadir)/GeographicLib
SCRIPTMANCMD = sed -e "s/@SCRIPT@/$*/g" \
	-e "s/@DATA@/`echo $* | cut -f3 -d-`/g" \
	-e "s%@GEOGRAPHICLIB_DATA@%$(geographiclib_data)%g" \
	$(srcdir)/script.8.in > $@

geographiclib-get-geoids.8:
	$(SCRIPTMANCMD)
geographiclib-get-gravity.8:
	$(SCRIPTMANCMD)
geographiclib-get-magnetic.8:
	$(SCRIPTMANCMD)

EXTRA_DIST = CMakeLists.txt $(MANPAGES) $(USAGE) $(HTMLMAN) script.8.in

maintainer-clean-local:
	rm -rf *.8
