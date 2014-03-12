Imports NETGeographicLib

Module example_LocalCartesian
    Sub Main()
        Try
            Dim earth As Geocentric = New Geocentric()
            Dim lat0 As Double = 48 + 50 / 60.0, lon0 = 2 + 20 / 60.0 ' Paris
            Dim proj As LocalCartesian = New LocalCartesian(lat0, lon0, 0, earth)
            ' Sample forward calculation
            Dim lat As Double = 50.9, lon = 1.8, h = 0 ' Calais
            Dim x, y, z As Double
            proj.Forward(lat, lon, h, x, y, z)
            Console.WriteLine(String.Format("{0} {1} {2}", x, y, z))
            ' Sample reverse calculation
            x = -38000.0 : y = 230000.0 : z = -4000.0
            proj.Reverse(x, y, z, lat, lon, h)
            Console.WriteLine(String.Format("{0} {1} {2}", lat, lon, h))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
