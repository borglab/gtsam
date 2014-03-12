Imports NETGeographicLib

Module example_TransverseMercatorExact
    Sub Main()
        Try
            Dim proj As TransverseMercatorExact = New TransverseMercatorExact() ' WGS84
            Dim lon0 As Double = -75          ' Central meridian for UTM zone 18
            ' Sample forward calculation
            Dim lat As Double = 40.3, lon = -74.7 ' Princeton, NJ
            Dim x, y As Double
            proj.Forward(lon0, lat, lon, x, y)
            Console.WriteLine(String.Format("{0} {1}", x, y))
            ' Sample reverse calculation
            x = 25000.0 : y = 4461000.0
            proj.Reverse(lon0, x, y, lat, lon)
            Console.WriteLine(String.Format("{0} {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
