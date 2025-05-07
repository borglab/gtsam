# Calling the GeographicLib C++ library from other languages

Here are some examples of calling the C++ library from other languages
such as C, Octave, and Python.

Although the geodesic capabilities of GeographicLib have been
implemented natively in several languages.  There are no plans to do the
same for its other capabilities since this leads to a large continuing
obligation for maintenance and documentation.  (Note however that the
Octave/MATLAB library includes additional capabilities, UTM, MGRS, etc.)

An alternative strategy is to call the C++ library directly from
another language, possibly via some "wrapper" routines.  This can be a
good strategy for a user who only wants to call a few GeographicLib
routines from another language.

Please contribute other examples, either for the languages given here or
for other languages.
