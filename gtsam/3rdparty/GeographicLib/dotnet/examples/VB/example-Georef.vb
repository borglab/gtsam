Imports NETGeographicLib
Module example_Georef
    Sub Main()
        Try
            ' Sample forward calculation
            Dim lat As Double = 57.64911, lon = 10.40744 ' Jutland
            Dim georefstring As String
            For prec1 As Integer = -1 To 11
                Georef.Forward(lat, lon, prec1, georefstring)
                Console.WriteLine(String.Format("Precision: {0} Georef: {1}", prec1, georefstring))
            Next
            ' Sample reverse calculation
            georefstring = "NKLN2444638946"
            Dim prec As Integer
            Georef.Reverse(georefstring.Substring(0, 2), lat, lon, prec, True)
            Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon))
            Georef.Reverse(georefstring.Substring(0, 4), lat, lon, prec, True)
            Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon))
            Georef.Reverse(georefstring, lat, lon, prec, True)
            Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught Exception {0}", ex.Message))
        End Try
    End Sub
End Module
