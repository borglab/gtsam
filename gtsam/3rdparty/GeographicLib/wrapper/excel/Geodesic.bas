Attribute VB_Name = "Geodesic"
Option Explicit

'   Declare the DLL functions

Private Declare PtrSafe Sub gdirect Lib "cgeodesic.dll" _
(ByVal lat1 As Double, ByVal lon1 As Double, _
 ByVal azi1 As Double, ByVal s12 As Double, _
 ByRef lat2 As Double, ByRef lon2 As Double, ByRef azi2 As Double)

Private Declare PtrSafe Sub ginverse Lib "cgeodesic.dll" _
(ByVal lat1 As Double, ByVal lon1 As Double, _
 ByVal lat2 As Double, ByVal lon2 As Double, _
 ByRef s12 As Double, ByRef azi1 As Double, ByRef azi2 As Double)

Private Declare PtrSafe Sub rdirect Lib "cgeodesic.dll" _
(ByVal lat1 As Double, ByVal lon1 As Double, _
 ByVal azi12 As Double, ByVal s12 As Double, _
 ByRef lat2 As Double, ByRef lon2 As Double)

Private Declare PtrSafe Sub rinverse Lib "cgeodesic.dll" _
(ByVal lat1 As Double, ByVal lon1 As Double, _
 ByVal lat2 As Double, ByVal lon2 As Double, _
 ByRef s12 As Double, ByRef azi12 As Double)

'   Define the custom worksheet functions that call the DLL functions

Function geodesic_direct_lat2(lat1 As Double, lon1 As Double, _
                              azi1 As Double, s12 As Double) As Double
  Attribute geodesic_direct_lat2.VB_Description = _
    "Solves direct geodesic problem for lat2."
  Dim lat2 As Double
  Dim lon2 As Double
  Dim azi2 As Double
  Call gdirect(lat1, lon1, azi1, s12, lat2, lon2, azi2)
  geodesic_direct_lat2 = lat2
End Function

Function geodesic_direct_lon2(lat1 As Double, lon1 As Double, _
                              azi1 As Double, s12 As Double) As Double
  Attribute geodesic_direct_lon2.VB_Description = _
    "Solves direct geodesic problem for lon2."
  Dim lat2 As Double
  Dim lon2 As Double
  Dim azi2 As Double
  Call gdirect(lat1, lon1, azi1, s12, lat2, lon2, azi2)
  geodesic_direct_lon2 = lon2
End Function

Function geodesic_direct_azi2(lat1 As Double, lon1 As Double, _
                              azi1 As Double, s12 As Double) As Double
  Attribute geodesic_direct_azi2.VB_Description = _
    "Solves direct geodesic problem for azi2."
  Dim lat2 As Double
  Dim lon2 As Double
  Dim azi2 As Double
  Call gdirect(lat1, lon1, azi1, s12, lat2, lon2, azi2)
  geodesic_direct_azi2 = azi2
End Function

Function geodesic_inverse_s12(lat1 As Double, lon1 As Double, _
                              lat2 As Double, lon2 As Double) As Double
  Attribute geodesic_inverse_s12.VB_Description = _
    "Solves inverse geodesic problem for s12."
  Dim s12 As Double
  Dim azi1 As Double
  Dim azi2 As Double
  Call ginverse(lat1, lon1, lat2, lon2, s12, azi1, azi2)
  geodesic_inverse_s12 = s12
End Function

Function geodesic_inverse_azi1(lat1 As Double, lon1 As Double, _
                               lat2 As Double, lon2 As Double) As Double
  Attribute geodesic_inverse_azi1.VB_Description = _
    "Solves inverse geodesic problem for azi1."
  Dim s12 As Double
  Dim azi1 As Double
  Dim azi2 As Double
  Call ginverse(lat1, lon1, lat2, lon2, s12, azi1, azi2)
  geodesic_inverse_azi1 = azi1
End Function

Function geodesic_inverse_azi2(lat1 As Double, lon1 As Double, _
                               lat2 As Double, lon2 As Double) As Double
  Attribute geodesic_inverse_azi2.VB_Description = _
    "Solves inverse geodesic problem for azi2."
  Dim s12 As Double
  Dim azi1 As Double
  Dim azi2 As Double
  Call ginverse(lat1, lon1, lat2, lon2, s12, azi1, azi2)
  geodesic_inverse_azi2 = azi2
End Function

Function rhumb_direct_lat2(lat1 As Double, lon1 As Double, _
                           azi12 As Double, s12 As Double) As Double
  Attribute rhumb_direct_lat2.VB_Description = _
    "Solves direct rhumb problem for lat2."
  Dim lat2 As Double
  Dim lon2 As Double
  Call rdirect(lat1, lon1, azi12, s12, lat2, lon2)
  rhumb_direct_lat2 = lat2
End Function

Function rhumb_direct_lon2(lat1 As Double, lon1 As Double, _
                           azi12 As Double, s12 As Double) As Double
  Attribute rhumb_direct_lon2.VB_Description = _
    "Solves direct rhumb problem for lon2."
  Dim lat2 As Double
  Dim lon2 As Double
  Call rdirect(lat1, lon1, azi12, s12, lat2, lon2)
  rhumb_direct_lon2 = lon2
End Function

Function rhumb_inverse_s12(lat1 As Double, lon1 As Double, _
                           lat2 As Double, lon2 As Double) As Double
  Attribute rhumb_inverse_s12.VB_Description = _
    "Solves inverse rhumb problem for s12."
  Dim s12 As Double
  Dim azi12 As Double
  Call rinverse(lat1, lon1, lat2, lon2, s12, azi12)
  rhumb_inverse_s12 = s12
End Function

Function rhumb_inverse_azi12(lat1 As Double, lon1 As Double, _
                             lat2 As Double, lon2 As Double) As Double
  Attribute rhumb_inverse_azi12.VB_Description = _
    "Solves inverse rhumb problem for azi12."
  Dim s12 As Double
  Dim azi12 As Double
  Call rinverse(lat1, lon1, lat2, lon2, s12, azi12)
  rhumb_inverse_azi12 = azi12
End Function
