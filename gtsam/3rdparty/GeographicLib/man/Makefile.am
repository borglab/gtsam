#
# Makefile.am
#
# Copyright (C) 2011, Charles Karney <charles@karney.com>

USAGE = \
	CartConvert.usage \
	ConicProj.usage \
	GeodesicProj.usage \
	GeoConvert.usage \
	GeodSolve.usage \
	GeoidEval.usage \
	Gravity.usage \
	MagneticField.usage \
	Planimeter.usage \
	TransverseMercatorProj.usage

MANPAGES = \
	CartConvert.1 \
	ConicProj.1 \
	GeodesicProj.1 \
	GeoConvert.1 \
	GeodSolve.1 \
	GeoidEval.1 \
	Gravity.1 \
	MagneticField.1 \
	Planimeter.1 \
	TransverseMercatorProj.1

HTMLMAN = \
	CartConvert.1.html \
	ConicProj.1.html \
	GeodesicProj.1.html \
	GeoConvert.1.html \
	GeodSolve.1.html \
	GeoidEval.1.html \
	Gravity.1.html \
	MagneticField.1.html \
	Planimeter.1.html \
	TransverseMercatorProj.1.html

POD2MAN = pod2man --center="GeographicLib Utilities" \
	--release="$(PACKAGE_STRING)"
PODFIX= sed -e 's%<head>%<head><link href="http://search.cpan.org/s/style.css" rel="stylesheet" type="text/css">%' -e 's%<code>\([^<>]*\)(\(.\))</code>%<a href="\1.\2.html">&</a>%'g
man1_MANS = $(MANPAGES)

SUFFIXES = .pod .1 .usage .1.html

all: man
man: manpages usage htmlman

manpages: $(MANPAGES)
usage: $(USAGE)
htmlman: $(HTMLMAN)

if HAVE_PODPROGS

.pod.usage:
	sh $(srcdir)/makeusage.sh $< $(VERSION) > $@

.pod.1:
	$(POD2MAN) $^ > $@

.pod.1.html:
	pod2html --noindex $^ | $(PODFIX) > $@

else

USAGECMD = cat $(srcdir)/$@ 2> /dev/null || \
	sed "s/@TOOL@/$*/g" $(srcdir)/dummy.usage.in > $@
MANCMD = cat $(srcdir)/$@ 2> /dev/null || \
	sed "s/@TOOL@/$*/g" $(srcdir)/dummy.1.in > $@
HTMLCMD = cat $(srcdir)/$@ 2> /dev/null || \
	sed "s/@TOOL@/$*/g" $(srcdir)/dummy.1.html.in > $@

CartConvert.usage:
	$(USAGECMD)
ConicProj.usage:
	$(USAGECMD)
GeodesicProj.usage:
	$(USAGECMD)
GeoConvert.usage:
	$(USAGECMD)
GeodSolve.usage:
	$(USAGECMD)
GeoidEval.usage:
	$(USAGECMD)
Gravity.usage:
	$(USAGECMD)
MagneticField.usage:
	$(USAGECMD)
Planimeter.usage:
	$(USAGECMD)
TransverseMercatorProj.usage:
	$(USAGECMD)

CartConvert.1:
	$(MANCMD)
ConicProj.1:
	$(MANCMD)
GeodesicProj.1:
	$(MANCMD)
GeoConvert.1:
	$(MANCMD)
GeodSolve.1:
	$(MANCMD)
GeoidEval.1:
	$(MANCMD)
Gravity.1:
	$(MANCMD)
MagneticField.1:
	$(MANCMD)
Planimeter.1:
	$(MANCMD)
TransverseMercatorProj.1:
	$(MANCMD)

CartConvert.1.html:
	$(HTMLCMD)
ConicProj.1.html:
	$(HTMLCMD)
GeodesicProj.1.html:
	$(HTMLCMD)
GeoConvert.1.html:
	$(HTMLCMD)
GeodSolve.1.html:
	$(HTMLCMD)
GeoidEval.1.html:
	$(HTMLCMD)
Gravity.1.html:
	$(HTMLCMD)
MagneticField.1.html:
	$(HTMLCMD)
Planimeter.1.html:
	$(HTMLCMD)
TransverseMercatorProj.1.html:
	$(HTMLCMD)

endif

EXTRA_DIST = Makefile.mk CMakeLists.txt makeusage.sh \
	GeoConvert.pod TransverseMercatorProj.pod \
	CartConvert.pod ConicProj.pod GeodSolve.pod GeodesicProj.pod \
	GeoidEval.pod Gravity.pod MagneticField.pod Planimeter.pod \
	$(MANPAGES) $(USAGE) $(HTMLMAN) \
	dummy.usage.in dummy.1.in dummy.1.html.in

maintainer-clean-local:
	rm -rf *.usage *.1.html *.1
