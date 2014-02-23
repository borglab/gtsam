/**
 * A test program for the GeographicLib.Geodesic.Direct method
 **********************************************************************/

import java.util.*;
import net.sf.geographiclib.*;
public class Direct {
/**
 * Solve the direct geodesic problem.
 *
 * This program reads in lines with lat1, lon1, azi1, s12 and prints out lines
 * with lat2, lon2, azi2 (for the WGS84 ellipsoid).
 **********************************************************************/
  public static void main(String[] args) {
    try {
      Scanner in = new Scanner(System.in);
      double lat1, lon1, azi1, s12;
      while (true) {
        lat1 = in.nextDouble(); lon1 = in.nextDouble();
        azi1 = in.nextDouble(); s12 = in.nextDouble();
        GeodesicData g = Geodesic.WGS84.Direct(lat1, lon1, azi1, s12);
        System.out.println(g.lat2 + " " + g.lon2 + " " + g.azi2);
      }
    }
    catch (Exception e) {}
  }
}
