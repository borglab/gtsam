#! /bin/sh -e
# Concatenate JavaScript files
HEADER=$1
shift
JS_VERSION=`grep -h "version_string = " "$@" | cut -f2 -d'"'`
FILE_INVENTORY=
for f; do
    FILE_INVENTORY="$FILE_INVENTORY `basename $f`"
done
sed -e "s/@JS_VERSION@/$JS_VERSION/" -e "s/@FILE_INVENTORY@/$FILE_INVENTORY/" \
    $HEADER
cat <<EOF

(function(cb) {
EOF
for f; do
    echo
    echo "/**************** `basename $f` ****************/"
    cat $f
done
cat <<EOF

cb(GeographicLib);

})(function(geo) {
  if (typeof module === 'object' && module.exports) {
    /******** support loading with node's require ********/
    module.exports = geo;
  } else if (typeof define === 'function' && define.amd) {
    /******** support loading with AMD ********/
    define('geographiclib', [], function() { return geo; });
  } else {
    /******** otherwise just pollute our global namespace ********/
    window.GeographicLib = geo;
  }
});
EOF
