Imports NETGeographicLib

Module example_MagneticCircle
    Sub Main()
        Try
            Dim mag As MagneticModel = New MagneticModel("wmm2010", "")
            Dim lat As Double = 27.99, lon0 = 86.93, h = 8820, t = 2012 ' Mt Everest
            ' Slow method of evaluating the values at several points on a circle of
            ' latitude.
            For i As Integer = -5 To 5
                Dim lon As Double = lon0 + i * 0.2
                Dim Bx, By, Bz As Double
                mag.Field(t, lat, lon, h, Bx, By, Bz)
                Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, Bx, By, Bz))
            Next
            ' Fast method of evaluating the values at several points on a circle of
            ' latitude using MagneticCircle.
            Dim circ As MagneticCircle = mag.Circle(t, lat, h)
            For i As Integer = -5 To 5
                Dim lon As Double = lon0 + i * 0.2
                Dim Bx, By, Bz As Double
                circ.Field(lon, Bx, By, Bz)
                Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, Bx, By, Bz))
            Next
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
