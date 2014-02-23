Imports NETGeographicLib

Module example_Ellipsoid
    Sub Main()
        Try
            Dim wgs84 As Ellipsoid = New Ellipsoid(Constants.WGS84.MajorRadius,
                                                   Constants.WGS84.Flattening)
            ' Alternatively: Dim wgs84 As Ellipsoid = new Ellipsoid()
            Console.WriteLine(String.Format(
                "The latitude half way between the equator and the pole is {0}",
                    wgs84.InverseRectifyingLatitude(45)))
            Console.WriteLine(String.Format(
                "Half the area of the ellipsoid lies between latitudes +/- {0}",
                wgs84.InverseAuthalicLatitude(30)))
            Console.WriteLine(String.Format(
                "The northernmost edge of a square Mercator map is at latitude {0}",
                wgs84.InverseIsometricLatitude(180)))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
