/**
 * Implementation of the net.sf.geographiclib.GeographicErr class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * Exception handling for GeographicLib.
 * <p>
 * A class to handle exceptions.  It's derived from RuntimeException so it
 * can be caught by the usual catch clauses.
 **********************************************************************/
public class GeographicErr extends RuntimeException {
  /**
   * Constructor
   * <p>
   * @param msg a string message, which is accessible in the catch
   *   clause via getMessage().
   **********************************************************************/
  public GeographicErr(String msg) { super(msg); }
}
