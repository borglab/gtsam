This is a Java implementation of the geodesic algorithms described in

  C. F. F. Karney,
  Algorithms for geodesics,
  J. Geodesy 87, 43-55 (2013);
  https://doi.org/10.1007/s00190-012-0578-z
  Addenda: https://geographiclib.sourceforge.io/geod-addenda.html

For documentation, see

  https://geographiclib.sourceforge.io/html/java/

The code in this directory is entirely self-contained.  In particular,
it does not depend on the C++ classes.  You can build the example
programs using, for example,

  cd inverse/src/main/java
  javac -cp .:../../../../src/main/java Inverse.java
  echo -30 0 29.5 179.5 | java -cp .:../../../../src/main/java Inverse

On Windows, change this to
  cd inverse\src\main\java
  javac -cp .;../../../../src/main/java Inverse.java
  echo -30 0 29.5 179.5 | java -cp .;../../../../src/main/java Inverse

Building with maven is also supported (see the documentation).
