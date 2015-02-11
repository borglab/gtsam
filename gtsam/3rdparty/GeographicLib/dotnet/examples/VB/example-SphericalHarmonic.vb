Imports NETGeographicLib

Module example_SphericalHarmonic
    Sub Main()
        Try
            Dim N As Integer = 3 ' The maximum degree
            Dim ca As Double() = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1} ' cosine coefficients
            Dim sa As Double() = {6, 5, 4, 3, 2, 1} ' sine coefficients
            Dim a As Double = 1
            Dim h As SphericalHarmonic = New SphericalHarmonic(ca, sa, N, a, SphericalHarmonic.Normalization.SCHMIDT)
            Dim x As Double = 2, y = 3, z = 1
            Dim vx, vy, vz As Double
            Dim v As Double = h.HarmonicSum(x, y, z, vx, vy, vz)
            Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
