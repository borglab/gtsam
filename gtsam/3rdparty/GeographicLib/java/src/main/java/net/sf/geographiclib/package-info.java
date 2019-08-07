/**
 * <h1>Geodesic routines from GeographicLib implemented in Java</h1>
 * @author Charles F. F. Karney (charles@karney.com)
 * @version 1.49
 *
 * <p>
 * The documentation for other versions is available at
 * <tt>https://geographiclib.sourceforge.io/m.nn/java</tt> for versions numbers
 * <tt>m.nn</tt> &ge; 1.31.
 * <p>
 * Licensed under the
 * <a href="http://www.opensource.org/licenses/MIT">MIT/X11 License</a>; see
 * <a href="https://geographiclib.sourceforge.io/html/LICENSE.txt">
 * LICENSE.txt</a>.
 *
 * <h2>Abstract</h2>
 * <p>
 * GeographicLib-Java is a Java implementation of the geodesic algorithms from
 * <a href="https://geographiclib.sourceforge.io">GeographicLib</a>.  This is a
 * self-contained library which makes it easy to do geodesic computations for
 * an ellipsoid of revolution in a Java program.  It requires Java version 1.2
 * or later.
 *
 * <h2>Downloading</h2>
 * <p>
 * Download either the source or the pre-built package as follows:
 *
 * <h3>Obtaining the source</h3>
 * GeographicLib-Java is part of GeographicLib which available for download at
 * <ul>
 * <li>
 *   <a href="https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.tar.gz">
 *   GeographicLib-1.49.tar.gz</a>
 * <li>
 *   <a href="https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.zip">
 *   GeographicLib-1.49.zip</a>
 * </ul>
 * <p>
 * as either a compressed tar file (tar.gz) or a zip file.  After unpacking
 * the source, the Java library can be found in GeographicLib-1.49/java.  (This
 * library is completely independent from the rest of GeodegraphicLib.)  The
 * library consists of the files in the src/main/java/net/sf/geographiclib
 * subdirectory.
 *
 * <h3>The pre-built package</h3>
 * GeographicLib-Java is available as a pre-built package on Maven Central
 * (thanks to Chris Bennight for help on this deployment).  So, if you use
 * <a href="http://maven.apache.org/">maven</a> to build your code, you just
 * need to include the dependency <pre>{@code
 *   <dependency>
 *     <groupId>net.sf.geographiclib</groupId>
 *     <artifactId>GeographicLib-Java</artifactId>
 *     <version>1.49</version>
 *   </dependency> }</pre>
 * in your {@code pom.xml}.
 *
 * <h2>Sample programs</h2>
 * <p>
 * Included with the source are 3 small test programs
 * <ul>
 * <li>
 *    {@code direct/src/main/java/Direct.java} is a simple command line utility
 *    for solving the direct geodesic problem;
 * <li>
 *    {@code inverse/src/main/java/Inverse.java} is a simple command line
 *    utility for solving the inverse geodesic problem;
 * <li>
 *    {@code planimeter/src/main/java/Planimeter.java} is a simple command line
 *    utility for computing the area of a geodesic polygon given its vertices.
 * </ul>
 * <p>
 * Here, for example, is {@code Inverse.java} <pre>{@code
 * // Solve the inverse geodesic problem.
 *
 * // This program reads in lines with lat1, lon1, lat2, lon2 and prints
 * // out lines with azi1, azi2, s12 (for the WGS84 ellipsoid).
 *
 * import java.util.*;
 * import net.sf.geographiclib.*;
 * public class Inverse {
 *   public static void main(String[] args) {
 *     try {
 *       Scanner in = new Scanner(System.in);
 *       double lat1, lon1, lat2, lon2;
 *       while (true) {
 *         lat1 = in.nextDouble(); lon1 = in.nextDouble();
 *         lat2 = in.nextDouble(); lon2 = in.nextDouble();
 *         GeodesicData g = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2);
 *         System.out.println(g.azi1 + " " + g.azi2 + " " + g.s12);
 *       }
 *     }
 *     catch (Exception e) {}
 *   }
 * }}</pre>
 *
 * <h2>Compiling and running a sample program</h2>
 * <p>
 * Three difference ways of compiling and running {@code Inverse.java} are
 * given.  These differ in the degree to which they utilize
 * <a href="http://maven.apache.org/">maven</a> to manage your Java code and
 * its dependencies.  (Thanks to Skip Breidbach for supplying the maven
 * support.)
 *
 * <h3>Without using maven</h3>
 * Compile and run as follows <pre>
 * cd inverse/src/main/java
 * javac -cp .:../../../../src/main/java Inverse.java
 * echo -30 0 29.5 179.5 | java -cp .:../../../../src/main/java Inverse </pre>
 * On Windows, change this to <pre>
 * cd inverse\src\main\java
 * javac -cp .;../../../../src/main/java Inverse.java
 * echo -30 0 29.5 179.5 | java -cp .;../../../../src/main/java Inverse </pre>
 *
 * <h3>Using maven to package GeographicLib</h3>
 * Use <a href="http://maven.apache.org/">maven</a> to create a jar file by
 * running (in the main java directory) <pre>
 * mvn package </pre>
 * (Your first run of maven may take a long time, because it needs to download
 * some additional packages to your local repository.)  Then compile and run
 * Inverse.java with <pre>
 * cd inverse/src/main/java
 * javac -cp .:../../../../target/GeographicLib-Java-1.49.jar Inverse.java
 * echo -30 0 29.5 179.5 |
 *   java -cp .:../../../../target/GeographicLib-Java-1.49.jar Inverse </pre>
 *
 * <h3>Using maven to build and run {@code Inverse.java}</h3>
 * The sample code includes a {@code pom.xml} which specifies
 * GeographicLib-Jave as a dependency.  You can build and install this
 * dependency by running (in the main java directory) <pre>
 * mvn install </pre>
 * Alternatively, you can let maven download it from Maven Central.  You can
 * compile and run Inverse.java with <pre>
 * cd inverse
 * mvn compile
 * echo -30 0 29.5 179.5 | mvn -q exec:java </pre>
 *
 * <h2>Using the library</h2>
 * <ul>
 * <li>
 *   Put <pre>
 *   import net.sf.geographiclib.*</pre>
 *   in your source code.
 * <li>
 *   Make calls to the geodesic routines from your code.
 * <li>
 *   Compile and run in one of the ways described above.
 * </ul>
 * <p>
 * The important classes are
 * <ul>
 * <li>
 *   {@link net.sf.geographiclib.Geodesic}, for direct and inverse geodesic
 *   calculations;
 * <li>
 *   {@link net.sf.geographiclib.GeodesicLine}, an efficient way of
 *   calculating multiple points on a single geodesic;
 * <li>
 *   {@link net.sf.geographiclib.GeodesicData}, the object containing the
 *   results of the geodesic calculations;
 * <li>
 *   {@link net.sf.geographiclib.GeodesicMask}, the constants that let you
 *   specify the variables to return in {@link
 *   net.sf.geographiclib.GeodesicData} and the capabilities of a {@link
 *   net.sf.geographiclib.GeodesicLine};
 * <li>
 *   {@link net.sf.geographiclib.Constants}, the parameters for the WGS84
 *   ellipsoid;
 * <li>
 *   {@link net.sf.geographiclib.PolygonArea}, a class to compute the
 *   perimeter and area of a geodesic polygon (returned as a {@link
 *   net.sf.geographiclib.PolygonResult}).
 * </ul>
 * <p>
 * The documentation is generated using javadoc when
 * {@code mvn package -P release} is run (the top of the documentation tree is
 * {@code target/apidocs/index.html}).  This is also available on the web at
 * <a href="https://geographiclib.sourceforge.io/html/java/index.html">
 * https://geographiclib.sourceforge.io/html/java/index.html</a>.
 *
 * <h2>External links</h2>
 * <ul>
 * <li>
 *   These algorithms are derived in C. F. F. Karney,
 *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
 *   Algorithms for geodesics</a>,
 *   J. Geodesy <b>87</b>, 43&ndash;55 (2013)
 *   (<a href="https://geographiclib.sourceforge.io/geod-addenda.html">addenda</a>).
 * <li>
 *   A longer paper on geodesics: C. F. F. Karney,
 *   <a href="https://arxiv.org/abs/1102.1215v1">Geodesics
 *   on an ellipsoid of revolution</a>,
 *   Feb. 2011
 *   (<a href="https://geographiclib.sourceforge.io/geod-addenda.html#geod-errata">
 *   errata</a>).
 * <li>
 *   <a href="https://geographiclib.sourceforge.io">
 *   The GeographicLib web site</a>.
 * <li>
 *   <a href="https://sourceforge.net/projects/geographiclib/">
 *     Main project page</a>
 * <li>
 *   <a href="https://sourceforge.net/p/geographiclib/code/ci/release/tree/">
 *     git repository</a>
 * <li>
 *   Implementations in various languages:
 *   <ul>
 *     <li>
 *       C++ (complete library):
 *       <a href="https://geographiclib.sourceforge.io/html/">
 *         documentation</a>,
 *       <a href="https://sourceforge.net/projects/geographiclib/files/distrib">
 *         download</a>
 *     <li>
 *       C (geodesic routines):
 *       <a href="https://geographiclib.sourceforge.io/html/C/">
 *         documentation</a>, also included with recent versions of
 *       <a href="https://github.com/OSGeo/proj.4/wiki">
 *         proj.4</a>
 *     <li>
 *       Fortran (geodesic routines):
 *       <a href="https://geographiclib.sourceforge.io/html/Fortran/">
 *         documentation</a>
 *     <li>
 *       Java (geodesic routines):
 *       <a href="http://repo1.maven.org/maven2/net/sf/geographiclib/GeographicLib-Java/">
 *         Maven Central package</a>,
 *       <a href="https://geographiclib.sourceforge.io/html/java/">
 *         documentation</a>
 *     <li>
 *       JavaScript (geodesic routines):
 *       <a href="https://www.npmjs.com/package/geographiclib">
 *         npm package</a>,
 *       <a href="https://geographiclib.sourceforge.io/html/js/">
 *         documentation</a>
 *     <li>
 *       Python (geodesic routines):
 *       <a href="http://pypi.python.org/pypi/geographiclib">
 *         PyPI package</a>,
 *       <a href="https://geographiclib.sourceforge.io/html/python/">
 *         documentation</a>
 *     <li>
 *       Matlab/Octave (geodesic and some other routines):
 *       <a href="http://www.mathworks.com/matlabcentral/fileexchange/50605">
 *         Matlab Central package</a>,
 *       <a href="http://www.mathworks.com/matlabcentral/fileexchange/50605/content/Contents.m">
 *         documentation</a>
 *     <li>
 *       C# (.NET wrapper for complete C++ library):
 *       <a href="https://geographiclib.sourceforge.io/html/NET/">
 *         documentation</a>.
 *   </ul>
 * <li>
 *   The section in the GeographicLib documentation on geodesics:
 *   <a href="https://geographiclib.sourceforge.io/html/geodesic.html">
 *   Geodesics on an ellipsoid of revolution</a>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/geodesic-papers/biblio.html">
 *   A geodesic bibliography</a>
 * <li>
 *   The wikipedia page,
 *   <a href="https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid">
 *   Geodesics on an ellipsoid</a>
 * </ul>
 *
 * <h2>Change log</h2>
 * <ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.47/java">Version 1.49</a>
 *   (released 2017-10-05)
 * <ul>
 * <li>
 *   Fix code formatting and add two tests.
 * </ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.47/java">Version 1.48</a>
 *   (released 2017-04-09)
 * <ul>
 * <li>
 *   Change default range for longitude and azimuth to
 *   (&minus;180&deg;, 180&deg;] (instead of [&minus;180&deg;, 180&deg;)).
 * </ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.47/java">Version 1.47</a>
 *   (released 2017-02-15)
 * <ul>
 * <li>
 *   Improve accuracy of area calculation (fixing a flaw introduced in
 *   version 1.46).
 * </ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.46/java">Version 1.46</a>
 *   (released 2016-02-15)
 * <ul>
 * <li>
 *   Fix bug where the wrong longitude was being returned with direct geodesic
 *   calculation with a negative distance when starting point was at a pole
 *   (this bug was introduced in version 1.44).
 * <li>
 *   Add Geodesic.DirectLine, Geodesic.ArcDirectLine, Geodesic.GenDirectLine,
 *   Geodesic.InverseLine, GeodesicLine.SetDistance, GeodesicLine.SetArc,
 *   GeodesicLine.GenSetDistance, GeodesicLine.Distance, GeodesicLine.Arc,
 *   GeodesicLine.GenDistance.
 * <li>
 *   More accurate inverse solution when longitude difference is close to
 *   180&deg;.
 * <li>
 *   GeoMath.AngDiff now returns a Pair.
 * </ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.45/java">Version 1.45</a>
 *   (released 2015-09-30)
 * <ul>
 * <li>
 *   The solution of the inverse problem now correctly returns NaNs if
 *   one of the latitudes is a NaN.
 * <li>
 *   Add implementation of the ellipsoidal
 *   {@link net.sf.geographiclib.Gnomonic} (courtesy of Sebastian Mattheis).
 * <li>
 *   Math.toRadians and Math.toDegrees are used instead of GeoMath.degree
 *   (which is now removed).  This requires Java 1.2 or later (released
 *   1998-12).
 * </ul>
 * <li>
 *   <a href="https://geographiclib.sourceforge.io/1.44/java">Version 1.44</a>
 *   (released 2015-08-14)
 * <ul>
 * <li>
 *   Improve accuracy of calculations by evaluating trigonometric
 *   functions more carefully and replacing the series for the reduced
 *   length with one with a smaller truncation error.
 * <li>
 *   The allowed ranges for longitudes and azimuths is now unlimited;
 *   it used to be [&minus;540&deg;, 540&deg;).
 * <li>
 *   Enforce the restriction of latitude to [&minus;90&deg;, 90&deg;] by
 *   returning NaNs if the latitude is outside this range.
 * <li>
 *   Geodesic.Inverse sets <i>s12</i> to zero for coincident points at pole
 *   (instead of returning a tiny quantity).
 * <li>
 *   Geodesic.Inverse pays attentions to the GeodesicMask.LONG_UNROLL bit in
 *   <i>outmask</i>.
 * </ul>
 * </ul>
 **********************************************************************/
package net.sf.geographiclib;
