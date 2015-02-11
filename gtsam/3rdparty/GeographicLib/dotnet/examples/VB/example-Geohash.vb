Imports NETGeographicLib

Module example_Geohash
    Sub Main()
        Try
            ' Sample forward calculation
            Dim lat As Double = 57.64911, lon = 10.40744 ' Jutland (the wikipedia example)
            Dim ghash As String
            Dim maxlen As Integer = Geohash.GeohashLength(0.00001)
            For len As Integer = 0 To maxlen
                Geohash.Forward(lat, lon, len, ghash)
                Console.WriteLine(ghash)
            Next
            ' Sample reverse calculation
            ghash = "u4pruydqqvj"
            For i As Integer = 0 To ghash.Length - 1
                Dim len As Integer
                Geohash.Reverse(ghash.Substring(0, i), lat, lon, len, True)
                Console.WriteLine(String.Format("Length: {0} Latitude: {1} Longitude: {2}", len, lat, lon))
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
