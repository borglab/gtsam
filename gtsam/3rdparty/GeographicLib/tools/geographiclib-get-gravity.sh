#! /bin/sh
#
# Download gravity models for use by GeographicLib::GravityModel.
#
# Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed
# under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

DEFAULTDIR="@GEOGRAPHICLIB_DATA@"
SUBDIR=gravity
NAME=gravity
MODEL=gravitymodel
CLASS=GravityModel
TOOL=Gravity
EXT=egm.cof
usage() {
    cat <<EOF
usage: $0 [-p parentdir] [-d] [-h] $MODEL...

This program downloads and installs the datasets used by the
GeographicLib::$CLASS class and the $TOOL tool to compute
gravity fields.  $MODEL is one of more of the names from this
table:

                       size (kB)
  name     degree    tar.bz2  disk
  egm84      18       27      26
  egm96      360     2100    2100
  egm2008   2190    76000   75000
  wgs84      20        1       1
  grs80      20        1       1

The size columns give the download and installed sizes of the datasets.
In addition you can specify

  all = all of the supported gravity models
  minimal = egm96 wgs84

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
egm84
egm96
egm2008
grs80
wgs84
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
		echo egm96; echo wgs84
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
