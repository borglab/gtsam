Here are some examples of calling the C++ library from C, MATLAB, and
python.

Although the geodesic capabilities of GeographicLib have been
implemented natively in several languages.  There are no plans to do the
same for its other capabilities since this leads to a large continuing
obligation for maintenance and documentation.  (Note however that thet
MATLAB/Octave library includes additional capabilities, UTM, MGRS, etc.)

An alternative strategy is to call the C++ library directly from another
language, possibly via some "wrapper" routines.  This done by
NETGeographicLib to provide access from C#.  This was relatively easy
because of the tight integration of C# and C++ with Visual Studio on
Windows systems.  Providing a similar interface for other languages is
challenging because the overall interface that GeographicLib exposes is
reasonably large and because the details often depend on the target
system.

Nevertheless, this can be a good strategy for a user who only want to
call a few GeographicLib routines from another language.

Please contribute other examples, either for the languages given here or
for other languages.
