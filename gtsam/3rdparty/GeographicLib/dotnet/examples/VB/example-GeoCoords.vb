Imports NETGeographicLib

Module example_GeoCoords
    Sub Main()
        Try
            ' Miscellaneous conversions
            Dim lat As Double = 33.3, lon = 44.4
            Dim c As GeoCoords = New GeoCoords(lat, lon, -1)
            Console.WriteLine(c.MGRSRepresentation(-3))
            c.Reset("18TWN0050", True, False)
            Console.WriteLine(c.DMSRepresentation(0, False, 0))
            Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", c.Latitude, c.Longitude))
            c.Reset("1d38'W 55d30'N", True, False)
            Console.WriteLine(c.GeoRepresentation(0, False))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
