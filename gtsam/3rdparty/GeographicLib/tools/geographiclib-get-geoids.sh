#! /bin/sh
#
# Download geoid datasets for use by GeographicLib::Geoid.  This is
# modeled on a similar script geographiclib-datasets-download by
# Francesco P. Lovergine <frankie@debian.org>
#
# Copyright (c) Charles Karney (2011-2013) <charles@karney.com> and
# licensed under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

DEFAULTDIR="@GEOGRAPHICLIB_DATA@"
SUBDIR=geoids
NAME=geoid
MODEL=geoid
CLASS=Geoid
TOOL=GeoidEval
EXT=pgm
usage() {
    cat <<EOF
usage: $0 [-p parentdir] [-d] [-h] $MODEL...

This program downloads and installs the datasets used by the
GeographicLib::$CLASS class and the $TOOL tool to compute geoid
heights.  These datasets are NGA earth gravity models evaluated on a
rectangular grid in latitude and longitude.  $MODEL is one of more of the
names from this table:

                                  size (MB)
  name         geoid    grid    tar.bz2  disk
  egm84-30     EGM84    30'      0.5      0.6
  egm84-15     EGM84    15'      1.5      2.1
  egm96-15     EGM96    15'      1.5      2.1
  egm96-5      EGM96     5'       11       19
  egm2008-5    EGM2008   5'       11       19
  egm2008-2_5  EGM2008   2.5'     35       75
  egm2008-1    EGM2008   1'      170      470

The size columns give the download and installed sizes of the datasets.
In addition you can specify

  all = all of the supported geoids
  minimal = emg96-5
  best = egm84-15 egm96-5 egm2008-1 (the highest resolution for each
         earth gravity model)
  good = same as best but substitute egm2008-2_5 for egm2008-1 to save
         on disk space

-p parentdir (default $DEFAULTDIR) specifies where the
datasets should be stored.  The "Default $NAME path" listed when running

  $TOOL -h

should be parentdir/$SUBDIR.  This script must be run by a user with
write access to this directory.

Normally only datasets which are not already in parentdir are
downloaded.  You can force the download and reinstallation with -f.

If -d is provided, the temporary directory which holds the downloads,
\$TMPDIR/$NAME-XXXXXXXX or ${TMPDIR:-/tmp}/$NAME-XXXXXXXX,
will be saved.  -h prints this help.

For more information on the $NAME datasets, visit

  https://geographiclib.sourceforge.io/html/$NAME.html

EOF
}

PARENTDIR="$DEFAULTDIR"
DEBUG=
FORCE=
while getopts hp:fd c; do
    case $c in
        h )
            usage;
            exit 0
            ;;
        p ) PARENTDIR="$OPTARG"
            ;;
	f ) FORCE=y
	    ;;
        d ) DEBUG=y
            ;;
        * )
            usage 1>&2;
            exit 1
            ;;
    esac
done
shift `expr $OPTIND - 1`
if test $# -eq 0; then
    usage 1>&2;
    exit 1
fi

test -d "$PARENTDIR"/$SUBDIR || mkdir -p "$PARENTDIR"/$SUBDIR 2> /dev/null
if test ! -d "$PARENTDIR"/$SUBDIR; then
    echo Cannot create directory $PARENTDIR/$SUBDIR 1>&2
    exit 1
fi

TEMP=
if test -z "$DEBUG"; then
trap 'trap "" 0; test "$TEMP" && rm -rf "$TEMP"; exit 1' 1 2 3 9 15
trap            'test "$TEMP" && rm -rf "$TEMP"'            0
fi
TEMP=`mktemp -d -q -t $NAME-XXXXXXXX`

if test -z "$TEMP" -o ! -d "$TEMP"; then
    echo Cannot create temporary directory 1>&2
    exit 1
fi

WRITETEST="$PARENTDIR"/$SUBDIR/write-test-`basename $TEMP`
if touch "$WRITETEST" 2> /dev/null; then
    rm -f "$WRITETEST"
else
    echo Cannot write in directory $PARENTDIR/$SUBDIR 1>&2
    exit 1
fi

set -e

cat > $TEMP/all <<EOF
egm84-30
egm84-15
egm96-15
egm96-5
egm2008-5
egm2008-2_5
egm2008-1
EOF

while test $# -gt 0; do
    if grep "^$1\$" $TEMP/all > /dev/null; then
	echo $1
    else
	case "$1" in
	    all )
		cat $TEMP/all
		;;
	    minimal )
		echo egm96-5
		;;
	    best )		# highest resolution models
		cat <<EOF
egm2008-1
egm96-5
egm84-15
EOF
		;;
	    good )	   # like best but with egm2008-1 -> egm2008-2_5
		cat <<EOF
egm2008-2_5
egm96-5
egm84-15
EOF
		;;
	    * )
		echo Unknown $MODEL $1 1>&2
		exit 1
		;;
	esac
    fi
    shift
done > $TEMP/list

sort -u $TEMP/list > $TEMP/todo

while read file; do
    if test -z "$FORCE" -a -s $PARENTDIR/$SUBDIR/$file.$EXT; then
	echo $PARENTDIR/$SUBDIR/$file.$EXT already installed, skipping $file...
	echo $file >> $TEMP/skip
	continue
    fi
    echo download $file.tar.bz2 ...
    echo $file >> $TEMP/download
    URL="https://downloads.sourceforge.net/project/geographiclib/$SUBDIR-distrib/$file.tar.bz2?use_mirror=autoselect"
    ARCHIVE=$TEMP/$file.tar.bz2
    wget -O$ARCHIVE $URL
    echo unpack $file.tar.bz2 ...
    tar vxojf $ARCHIVE -C $PARENTDIR
    echo $MODEL $file installed.
done < $TEMP/todo

if test "$DEBUG"; then
    echo Saving temporary directory $TEMP
fi
echo
if test -s $TEMP/download; then
    n=`wc -l < $TEMP/download`
    s=; test $n -gt 1 && s=s
    cat <<EOF
Installed $NAME dataset$s `tr '\n' ' ' < $TEMP/download`in $PARENTDIR/$SUBDIR.
EOF
fi
if test -s $TEMP/skip; then
    n=`wc -l < $TEMP/skip`
    s=; test $n -gt 1 && s=s
    cat <<EOF
Skipped $NAME dataset$s `tr '\n' ' ' < $TEMP/skip | sed 's/ $//'`.
EOF
fi
echo
