/**
 * A test program for the GeographicLib.PolygonArea class
 **********************************************************************/

import java.util.*;
import net.sf.geographiclib.*;
/**
 * Compute the area of a geodesic polygon.
 *
 * This program reads lines with lat, lon for each vertex of a polygon.  At the
 * end of input, the program prints the number of vertices, the perimeter of
 * the polygon and its area (for the WGS84 ellipsoid).
 **********************************************************************/
public class Planimeter {
  public static void main(String[] args) {
    PolygonArea p = new PolygonArea(Geodesic.WGS84, false);
    try {
      Scanner in = new Scanner(System.in);
      while (true) {
        double lat = in.nextDouble(), lon = in.nextDouble();
        p.AddPoint(lat, lon);
      }
    }
    catch (Exception e) {}
    PolygonResult r = p.Compute();
    System.out.println(r.num + " " + r.perimeter + " " + r.area);
  }
}
