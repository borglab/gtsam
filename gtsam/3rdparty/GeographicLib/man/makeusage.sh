#! /bin/sh
# Convert a pod file into a usage function for the GeographicLib utilities.

SOURCE=$1
NAME=`basename $SOURCE .pod`
VERSION=$2

(
cat<<EOF
int usage(int retval, bool brief) {
  if (brief)
    ( retval ? std::cerr : std::cout ) << "Usage:\n\\
EOF

pod2man $SOURCE | nroff -c -man 2>/dev/null | col -b -x |
sed -e 1,/SYNOPSIS/d -e '/^$/,$d' -e 's/  / /g' -e 's/$/\\n\\/' -e 's/"/\\"/g'

cat <<EOF
\n\\
For full documentation type:\n\\
    $NAME --help\n\\
or visit:\n\\
    https://geographiclib.sourceforge.io/$VERSION/$NAME.1.html\n";
  else
    ( retval ? std::cerr : std::cout ) << "Man page:\n\\
EOF

pod2man $SOURCE | nroff -c -man 2>/dev/null | col -b -x | head --lines -4 |
tail --lines +5 | sed -e 's/\\/\\\\/g' -e 's/$/\\n\\/' -e 's/"/\\"/g'

cat <<EOF
";
  return retval;
}
EOF
) | # Break long strings assuming that files don't contain the ~ character
tr '\n' '~' | sed -e's/\\~/"~"/g' -e's/""//g' | tr '~' '\n'
