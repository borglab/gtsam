Imports NETGeographicLib

Module example_SphericalHarmonic1
    Sub Main()
        Try
            Dim N As Integer = 3, N1 = 2                  ' The maximum degrees
            Dim ca As Double() = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1} ' cosine coefficients
            Dim sa As Double() = {6, 5, 4, 3, 2, 1} ' sine coefficients
            Dim cb As Double() = {1, 2, 3, 4, 5, 6}
            Dim sb As Double() = {3, 2, 1}
            Dim a As Double = 1
            Dim h As SphericalHarmonic1 = New SphericalHarmonic1(ca, sa, N, cb, sb, N1, a, SphericalHarmonic1.Normalization.SCHMIDT)
            Dim tau As Double = 0.1, x = 2, y = 3, z = 1
            Dim vx, vy, vz As Double
            Dim v As Double = h.HarmonicSum(tau, x, y, z, vx, vy, vz)
            Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
