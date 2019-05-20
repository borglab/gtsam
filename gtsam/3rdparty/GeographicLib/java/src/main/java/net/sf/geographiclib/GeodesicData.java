/**
 * Implementation of the net.sf.geographiclib.GeodesicData class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * The results of geodesic calculations.
 *
 * This is used to return the results for a geodesic between point 1
 * (<i>lat1</i>, <i>lon1</i>) and point 2 (<i>lat2</i>, <i>lon2</i>).  Fields
 * that have not been set will be filled with Double.NaN.  The returned
 * GeodesicData objects always include the parameters provided to {@link
 * Geodesic#Direct(double, double, double, double) Geodesic.Direct} and {@link
 * Geodesic#Inverse(double, double, double, double) Geodesic.Inverse} and it
 * always includes the field <i>a12</i>.
 **********************************************************************/
public class GeodesicData {
  /**
   * latitude of point 1 (degrees).
   **********************************************************************/
  public double lat1;
  /**
   * longitude of point 1 (degrees).
   **********************************************************************/
  public double lon1;
  /**
   * azimuth at point 1 (degrees).
   **********************************************************************/
  public double azi1;
  /**
   * latitude of point 2 (degrees).
   **********************************************************************/
  public double lat2;
  /**
   * longitude of point 2 (degrees).
   **********************************************************************/
  public double lon2;
  /**
   * azimuth at point 2 (degrees).
   **********************************************************************/
  public double azi2;
  /**
   * distance between point 1 and point 2 (meters).
   **********************************************************************/
  public double s12;
  /**
   * arc length on the auxiliary sphere between point 1 and point 2
   *   (degrees).
   **********************************************************************/
  public double a12;
  /**
   * reduced length of geodesic (meters).
   **********************************************************************/
  public double m12;
  /**
   * geodesic scale of point 2 relative to point 1 (dimensionless).
   **********************************************************************/
  public double M12;
  /**
   * geodesic scale of point 1 relative to point 2 (dimensionless).
   **********************************************************************/
  public double M21;
  /**
   * area under the geodesic (meters<sup>2</sup>).
   **********************************************************************/
  public double S12;
  /**
   * Initialize all the fields to Double.NaN.
   **********************************************************************/
  public GeodesicData() {
    lat1 = lon1 = azi1 = lat2 = lon2 = azi2 =
      s12 = a12 = m12 = M12 = M21 = S12 = Double.NaN;
  }
}
