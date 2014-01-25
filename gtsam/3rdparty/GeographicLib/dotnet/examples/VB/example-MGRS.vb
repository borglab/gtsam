Imports NETGeographicLib

Module example_MGRS
    Sub Main()
        Try
            ' See also example-GeoCoords.cpp
            ' Sample forward calculation
            Dim lat As Double = 33.3, lon = 44.4 ' Baghdad
            Dim zone As Integer
            Dim northp As Boolean
            Dim x, y As Double
            UTMUPS.Forward(lat, lon, zone, northp, x, y, -1, True)
            Dim mgrsStr As String
            MGRS.Forward(zone, northp, x, y, lat, 5, mgrsStr)
            Console.WriteLine(mgrsStr)
            ' Sample reverse calculation
            mgrsStr = "38SMB4488"
            Dim prec As Integer
            MGRS.Reverse(mgrsStr, zone, northp, x, y, prec, True)
            UTMUPS.Reverse(zone, northp, x, y, lat, lon, True)
            Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
