Imports NETGeographicLib

Module example_AlbersEqualArea
    Sub Main()
        Try
            Dim lat1 As Double = 40 + 58 / 60.0 : Dim lat2 As Double = 39 + 56 / 60.0 ' standard parallels
            Dim k1 As Double = 1  ' scale
            Dim lon0 As Double = -77 - 45 / 60.0 ' Central meridian
            ' Set up basic projection
            Dim albers As AlbersEqualArea = New AlbersEqualArea(Constants.WGS84.MajorRadius,
                                                                Constants.WGS84.Flattening,
                                                                lat1, lat2, k1)
            ' Sample conversion from geodetic to Albers Equal Area
            Dim lat As Double = 39.95 : Dim lon As Double = -75.17  ' Philadelphia
            Dim x, y As Double
            albers.Forward(lon0, lat, lon, x, y)
            Console.WriteLine(String.Format("X: {0} Y: {1}", x, y))
            ' Sample conversion from Albers Equal Area grid to geodetic
            x = 220000.0 : y = -53000.0
            albers.Reverse(lon0, x, y, lat, lon)
            Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
