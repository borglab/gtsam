Imports NETGeographicLib

Module example_GravityCircle
    Sub Main()
        Try
            Dim grav As GravityModel = New GravityModel("egm96", "")
            Dim lat As Double = 27.99, lon0 = 86.93, h = 8820 ' Mt Everest
            ' Slow method of evaluating the values at several points on a circle of
            ' latitude.
            For i As Integer = -5 To 5
                Dim lon As Double = lon0 + i * 0.2
                Dim gx, gy, gz As Double
                grav.Gravity(lat, lon, h, gx, gy, gz)
                Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, gx, gy, gz))
            Next
            ' Fast method of evaluating the values at several points on a circle of
            ' latitude using GravityCircle.
            Dim circ As GravityCircle = grav.Circle(lat, h, GravityModel.Mask.ALL)
            For i As Integer = -5 To 5
                Dim lon As Double = lon0 + i * 0.2
                Dim gx, gy, gz As Double
                circ.Gravity(lon, gx, gy, gz)
                Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, gx, gy, gz))
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
