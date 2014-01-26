Imports NETGeographicLib

Module example_GravityModel
    Sub Main()
        Try
            Dim grav As GravityModel = New GravityModel("egm96", "")
            Dim lat As Double = 27.99, lon = 86.93, h = 8820 ' Mt Everest
            Dim gx, gy, gz As Double
            grav.Gravity(lat, lon, h, gx, gy, gz)
            Console.WriteLine(String.Format("{0} {1} {2}", gx, gy, gz))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
