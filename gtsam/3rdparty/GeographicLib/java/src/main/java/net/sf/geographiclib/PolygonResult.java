/**
 * Implementation of the net.sf.geographiclib.PolygonResult class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * A container for the results from PolygonArea.
 **********************************************************************/
public class PolygonResult {
  /**
   * The number of vertices in the polygon
   **********************************************************************/
  public int num;
  /**
   * The perimeter of the polygon or the length of the polyline (meters).
   **********************************************************************/
  public double perimeter;
  /**
   * The area of the polygon (meters<sup>2</sup>).
   **********************************************************************/
  public double area;
  /**
   * Constructor
   * <p>
   * @param num the number of vertices in the polygon.
   * @param perimeter the perimeter of the polygon or the length of the
   *   polyline (meters).
   * @param area the area of the polygon (meters<sup>2</sup>).
   **********************************************************************/
  public PolygonResult(int num, double perimeter, double area) {
    this.num = num;
    this.perimeter = perimeter;
    this.area = area;
  }
}
