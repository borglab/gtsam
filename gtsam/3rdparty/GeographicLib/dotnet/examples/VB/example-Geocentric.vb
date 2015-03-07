Imports NETGeographicLib

Module example_Geocentric
    Sub Main()
        Try
            Dim earth As Geocentric = New Geocentric(Constants.WGS84.MajorRadius,
                                                     Constants.WGS84.Flattening)
            ' Alternatively: Geocentric earth = new Geocentric();
            ' Sample forward calculation
            Dim lat As Double = 27.99, lon = 86.93, h = 8820 ' Mt Everest
            Dim X, Y, Z As Double
            earth.Forward(lat, lon, h, X, Y, Z)
            Console.WriteLine(String.Format("{0} {1} {2}",
                Math.Floor(X / 1000 + 0.5),
                Math.Floor(Y / 1000 + 0.5),
                Math.Floor(Z / 1000 + 0.5)))
            ' Sample reverse calculation
            X = 302000.0 : Y = 5636000.0 : Z = 2980000.0
            earth.Reverse(X, Y, Z, lat, lon, h)
            Console.WriteLine(String.Format("{0} {1} {2}", lat, lon, h))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
