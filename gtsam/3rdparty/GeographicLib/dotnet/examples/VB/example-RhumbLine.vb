Imports NETGeographicLib
Module example_RhumbLine
    Sub Main()
        Try
            ' Print waypoints between JFK and SIN
            Dim rhumb As Rhumb = New Rhumb(Constants.WGS84.MajorRadius, Constants.WGS84.Flattening, True)
            ' Alternatively: const Rhumb& rhumb = Rhumb::WGS84();
            Dim lat1 As Double = 40.64, lon1 = -73.779 ' JFK
            Dim lat2 As Double = 1.359, lon2 = 103.989 ' SIN
            Dim s12 As Double, azi12
            rhumb.Inverse(lat1, lon1, lat2, lon2, s12, azi12);
            Dim line As RhumbLine = rhumb.Line(lat1, lon1, azi12)

            Dim ds As Double = 500000.0          ' Nominal distance between points = 500 km
            Dim num As Integer = (Integer)Math.Ceiling(s12 / ds) ' The number of intervals
            ' Use intervals of equal length
            ds = s12 / num
            For i As Integer = 0 To num - 1
                Dim lat As Double, lon
                line.Position(i * ds, lat, lon)
                Console.WriteLine("{0} {1} {2}", i, lat, lon)
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
