/**
 * Implementation of the net.sf.geographiclib.Pair class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * A pair of double precision numbers.
 * <p>
 * This duplicates the C++ class {@code std::pair<double, double>}.
 **********************************************************************/
public class Pair {
  /**
   * The first member of the pair.
   **********************************************************************/
  public double first;
  /**
   * The second member of the pair.
   **********************************************************************/
  public double second;
  /**
   * Constructor
   * <p>
   * @param first the first member of the pair.
   * @param second the second member of the pair.
   **********************************************************************/
  public Pair(double first, double second)
  { this.first = first; this.second = second; }
}
