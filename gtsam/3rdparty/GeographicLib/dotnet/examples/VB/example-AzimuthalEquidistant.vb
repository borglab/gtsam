Imports NETGeographicLib

Module example_AzimuthalEquidistant
    Sub Main()
        Try
            Dim geod As Geodesic = New Geodesic() ' WGS84
            Dim lat0 As Double = 48 + 50 / 60.0, lon0 = 2 + 20 / 60.0 ' Paris
            Dim proj As AzimuthalEquidistant = New AzimuthalEquidistant(geod)
            ' Sample forward calculation
            Dim lat As Double = 50.9, lon = 1.8 ' Calais
            Dim x, y As Double
            proj.Forward(lat0, lon0, lat, lon, x, y)
            Console.WriteLine(String.Format("X: {0} Y: {1}", x, y))
            ' Sample reverse calculation
            x = -38000.0 : y = 230000.0
            proj.Reverse(lat0, lon0, x, y, lat, lon)
            Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
