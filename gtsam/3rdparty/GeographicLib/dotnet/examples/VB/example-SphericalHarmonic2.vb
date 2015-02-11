Imports NETGeographicLib

Module example_SphericalHarmonic2
    Sub Main()
        Try
            Dim N As Integer = 3, N1 = 2, N2 = 1 ' The maximum degrees
            Dim ca As Double() = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1} ' cosine coefficients
            Dim sa As Double() = {6, 5, 4, 3, 2, 1} ' sine coefficients
            Dim cb As Double() = {1, 2, 3, 4, 5, 6}
            Dim sb As Double() = {3, 2, 1}
            Dim cc As Double() = {2, 1}
            Dim S2 As Double() = {0}
            Dim a As Double = 1
            Dim h As SphericalHarmonic2 = New SphericalHarmonic2(
                                    ca, sa, N, N, N, cb, sb, N1, N1, N1,
                                    cc, S2, N2, N2, 0, a,
                                    SphericalHarmonic2.Normalization.SCHMIDT)
            Dim tau1 As Double = 0.1, tau2 = 0.05, x = 2, y = 3, z = 1
            Dim vx, vy, vz As Double
            Dim v As Double = h.HarmonicSum(tau1, tau2, x, y, z, vx, vy, vz)
            Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
