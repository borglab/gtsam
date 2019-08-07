Imports NETGeographicLib

Module example_UTMUPS
    Sub Main()
        Try
            ' See also example-GeoCoords.cpp
            ' Sample forward calculation
            Dim lat As Double = 33.3, lon = 44.4 ' Baghdad
            Dim zone As Integer
            Dim northp As Boolean
            Dim x, y As Double
            UTMUPS.Forward(lat, lon, zone, northp, x, y, -1, True)
            Dim zonestr As String = UTMUPS.EncodeZone(zone, northp, True)
            Console.WriteLine(String.Format("{0} {1} {2}", zonestr, x, y))
            ' Sample reverse calculation
            zonestr = "38N"
            UTMUPS.DecodeZone(zonestr, zone, northp)
            x = 444000.0 : y = 3688000.0
            UTMUPS.Reverse(zone, northp, x, y, lat, lon, True)
            Console.WriteLine(String.Format("{0} {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
