Imports NETGeographicLib

Module example_MagneticModel
    Sub Main()
        Try
            Dim mag As MagneticModel = New MagneticModel("wmm2010", "")
            Dim lat As Double = 27.99, lon = 86.93, h = 8820, t = 2012 ' Mt Everest
            Dim Bx, By, Bz As Double
            mag.Field(t, lat, lon, H, Bx, By, Bz)
            Dim bigH, F, D, I As Double
            MagneticModel.FieldComponents(Bx, By, Bz, bigH, F, D, I)
            Console.WriteLine(String.Format("{0} {1} {2} {3}", bigH, F, D, I))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
