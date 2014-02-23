/**
 * A test program for the GeographicLib.Geodesic.Inverse method
 **********************************************************************/

import java.util.*;
import net.sf.geographiclib.*;
/**
 * Solve the inverse geodesic problem.
 *
 * This program reads in lines with lat1, lon1, lat2, lon2 and prints out lines
 * with azi1, azi2, s12 (for the WGS84 ellipsoid).
 **********************************************************************/
public class Inverse {
  public static void main(String[] args) {
    try {
      Scanner in = new Scanner(System.in);
      double lat1, lon1, lat2, lon2;
      while (true) {
        lat1 = in.nextDouble(); lon1 = in.nextDouble();
        lat2 = in.nextDouble(); lon2 = in.nextDouble();
        GeodesicData g = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2);
        System.out.println(g.azi1 + " " + g.azi2 + " " + g.s12);
      }
    }
    catch (Exception e) {}
  }
}
