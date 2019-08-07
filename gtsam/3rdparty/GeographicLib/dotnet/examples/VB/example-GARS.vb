Imports NETGeographicLib
Module example_GARS
    Sub Main()
        Try
            ' Sample forward calculation
            Dim lat As Double = 57.64911, lon = 10.40744
            Dim garstring As String
            For prec As Integer = 0 To 2
                GARS.Forward(lat, lon, prec, garstring)
                Console.WriteLine(String.Format("Precision: {0} GARS: {1}", prec, garstring))
            Next
            ' Sample reverse calculation
            garstring = "381NH45"
            For len As Integer = 5 To garstring.Length
                Dim prec As Integer
                GARS.Reverse(garstring.Substring(0, len), lat, lon, prec, True)
                Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude {2}", prec, lat, lon))
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught Exception {0}", ex.Message))
        End Try
    End Sub
End Module
