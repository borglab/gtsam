Imports NETGeographicLib

Module example_CircularEngine
    Sub Main()
        ' This computes the same value as example-SphericalHarmonic.cpp using a
        ' CircularEngine (which will be faster if many values on a circle of
        ' latitude are to be found).
        Try
            Dim N As Integer = 3 ' The maxium degree
            Dim ca As Double() = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1} ' cosine coefficients
            Dim sa As Double() = {6, 5, 4, 3, 2, 1} ' sine coefficients
            Dim a As Double = 1
            Dim h As SphericalHarmonic = New SphericalHarmonic(ca, sa, N, a, SphericalHarmonic.Normalization.SCHMIDT)
            Dim x As Double = 2, y = 3, z = 1, p = Math.Sqrt(x * x + y * y)
            Dim circ As CircularEngine = h.Circle(p, z, True)
            Dim v, vx, vy, vz As Double
            v = circ.LongitudeSum(x / p, y / p, vx, vy, vz)
            Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
