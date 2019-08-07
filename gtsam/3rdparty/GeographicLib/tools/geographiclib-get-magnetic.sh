#! /bin/sh
#
# Download magnetic models for use by GeographicLib::MagneticModel.
#
# Copyright (c) Charles Karney (2011-2015) <charles@karney.com> and
# licensed under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

DEFAULTDIR="@GEOGRAPHICLIB_DATA@"
SUBDIR=magnetic
NAME=magnetic
MODEL=magneticmodel
CLASS=MagneticModel
TOOL=MagneticField
EXT=wmm.cof
usage() {
    cat <<EOF
usage: $0 [-p parentdir] [-d] [-h] $MODEL...

This program downloads and installs the datasets used by the
GeographicLib::$CLASS class and the $TOOL tool to compute
magnetic fields.  $MODEL is one of more of the names from this
table:

                                  size (kB)
  name     degree    years      tar.bz2  disk
  wmm2010    12    2010-2015      2       3
  wmm2015    12    2015-2020      2       3
  igrf11     13    1900-2015      7      25
  igrf12     13    1900-2020      7      26
  emm2010   739    2010-2015    3700    4400
  emm2015   729    2000-2020     660    4300
  emm2017   790    2000-2022    1740    5050

The size columns give the download and installed sizes of the datasets.
In addition you can specify

  all = all of the supported magnetic models
  minimal = wmm2015 igrf12

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

For more information on the magnetic models, visit

  https://geographiclib.sourceforge.io/html/$NAME.html

EOF
}

PARENTDIR="$DEFAULTDIR"
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
wmm2010
wmm2015
emm2010
emm2015
emm2017
igrf11
igrf12
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
		echo wmm2015; echo igrf12
		;;
	    * )
		echo Unknown magnetic model $1 1>&2
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
