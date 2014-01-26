"""geodesiccapability.py: capability constants for geodesic{,line}.py"""
# geodesiccapability.py
#
# This gathers the capability constants need by geodesic.py and
# geodesicline.py.  See the documentation for the GeographicLib::Geodesic class
# for more information at
#
#    http://geographiclib.sourceforge.net/html/annotated.html
#
# Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed under
# the MIT/X11 License.  For more information, see
# http://geographiclib.sourceforge.net/
######################################################################

class GeodesicCapability(object):
  """
  Capability constants shared between Geodesic and GeodesicLine.
  """

  CAP_NONE = 0
  CAP_C1   = 1 << 0
  CAP_C1p  = 1 << 1
  CAP_C2   = 1 << 2
  CAP_C3   = 1 << 3
  CAP_C4   = 1 << 4
  CAP_ALL  = 0x1F
  OUT_ALL  = 0x7F80
  EMPTY         = 0
  LATITUDE      = 1 << 7  | CAP_NONE
  LONGITUDE     = 1 << 8  | CAP_C3
  AZIMUTH       = 1 << 9  | CAP_NONE
  DISTANCE      = 1 << 10 | CAP_C1
  DISTANCE_IN   = 1 << 11 | CAP_C1 | CAP_C1p
  REDUCEDLENGTH = 1 << 12 | CAP_C1 | CAP_C2
  GEODESICSCALE = 1 << 13 | CAP_C1 | CAP_C2
  AREA          = 1 << 14 | CAP_C4
  ALL           = OUT_ALL | CAP_ALL

