Imports NETGeographicLib

Module example_PolarStereographic
    Sub Main()
        Try
            Dim proj As PolarStereographic = New PolarStereographic() ' WGS84
            Dim northp As Boolean = True
            ' Sample forward calculation
            Dim lat As Double = 61.2, lon = -149.9 ' Anchorage
            Dim x, y As Double
            proj.Forward(northp, lat, lon, x, y)
            Console.WriteLine(String.Format("{0} {1}", x, y))
            ' Sample reverse calculation
            x = -1637000.0 : y = 2824000.0
            proj.Reverse(northp, x, y, lat, lon)
            Console.WriteLine(String.Format("{0} {1}", lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
