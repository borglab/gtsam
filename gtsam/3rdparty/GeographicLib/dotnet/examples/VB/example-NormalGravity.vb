Imports NETGeographicLib

Module example_NormalGravity
    Sub Main()
        Try
            Dim grav As NormalGravity = New NormalGravity(NormalGravity.StandardModels.WGS84)
            Dim lat As Double = 27.99, h = 8820 ' Mt Everest
            Dim gammay, gammaz As Double
            grav.Gravity(lat, h, gammay, gammaz)
            Console.WriteLine(String.Format("{0} {1}", gammay, gammaz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
