Imports NETGeographicLib

Module example_GeodesicLineExact
    Sub Main()
        Try
            ' Print waypoints between JFK and SIN
            Dim geod As GeodesicExact = New GeodesicExact() ' WGS84
            Dim lat1 As Double = 40.64, lon1 = -73.779 ' JFK
            Dim lat2 As Double = 1.359, lon2 = 103.989 ' SIN
            Dim s12, azi1, azi2 As Double
            Dim a12 As Double = geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2)
            Dim line As GeodesicLineExact = New GeodesicLineExact(geod, lat1, lon1, azi1, Mask.ALL)
            ' Alternatively Dim line As GeodesicLineExact = geod.Line(lat1, lon1, azi1, Mask.ALL)
            Dim ds As Double = 500000.0 ' Nominal distance between points = 500 km
            Dim num As Integer = CInt(Math.Ceiling(s12 / ds)) ' The number of intervals
            ' Use intervals of equal length
            ds = s12 / num
            For i As Integer = 0 To num
                Dim lat, lon As Double
                line.Position(i * ds, lat, lon)
                Console.WriteLine(String.Format("i: {0} Latitude: {1} Longitude: {2}", i, lat, lon))
            Next
            ' Slightly faster, use intervals of equal arc length
            Dim da As Double = a12 / num
            For i As Integer = 0 To num
                Dim lat, lon As Double
                line.ArcPosition(i * da, lat, lon)
                Console.WriteLine(String.Format("i: {0} Latitude: {1} Longitude: {2}", i, lat, lon))
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
