Imports NETGeographicLib

Module example_LambertConformalConic
    Sub Main()
        Try
            ' Define the Pennsylvania South state coordinate system EPSG:3364
            ' http://www.spatialreference.org/ref/epsg/3364/
            Dim lat1 As Double = 40 + 58 / 60.0, lat2 = 39 + 56 / 60.0  ' standard parallels
            Dim k1 As Double = 1                                        ' scale
            Dim lat0 As Double = 39 + 20 / 60.0, lon0 = -77 - 45 / 60.0 ' origin
            Dim fe As Double = 600000, fn = 0                           ' false easting and northing
            ' Set up basic projection
            Dim PASouth As LambertConformalConic = New LambertConformalConic(Constants.WGS84.MajorRadius,
                                                                             Constants.WGS84.Flattening,
                                                                             lat1, lat2, k1)
            Dim x0, y0 As Double
            ' Transform origin point
            PASouth.Forward(lon0, lat0, lon0, x0, y0)
            x0 -= fe : y0 -= fn
            ' Sample conversion from geodetic to PASouth grid
            Dim lat As Double = 39.95, lon = -75.17    ' Philadelphia
            Dim x, y As Double
            PASouth.Forward(lon0, lat, lon, x, y)
            x -= x0 : y -= y0
            Console.WriteLine(String.Format("{0} {1}", x, y))
            ' Sample conversion from PASouth grid to geodetic
            x = 820000.0 : y = 72000.0
            x += x0 : y += y0
            PASouth.Reverse(lon0, x, y, lat, lon)
            Console.WriteLine(String.Format("{0} {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
