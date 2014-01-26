Imports NETGeographicLib

Module example_Geoid
    Sub Main()
        Try
            Dim egm96 As Geoid = New Geoid("egm96-5", "", True, False)
            ' Convert height above egm96 to height above the ellipsoid
            Dim lat As Double = 42, lon = -75, height_above_geoid = 20
            Dim geoid_height As Double = egm96.Height(lat, lon)
            Dim height_above_ellipsoid As Double  = (height_above_geoid +
                    CDbl(Geoid.ConvertFlag.GEOIDTOELLIPSOID) * geoid_height)
            Console.WriteLine(height_above_ellipsoid)
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
