Imports NETGeographicLib

Module example_Rhumb
    Sub Main()
        Try
            Dim rhumb As Rhumb = New Rhumb(Constants.WGS84.MajorRadius, Constants.WGS84.Flattening, True)
            ' Alternatively: const Rhumb& rhumb = Rhumb::WGS84();

            ' Sample direct calculation, travelling about NE from JFK
            Dim lat1 As Double = 40.6, lon1 = -73.8, s12 = 5500000.0, azi12 = 51
            Dim lat2 As Double, lon2
            rhumb.Direct(lat1, lon1, azi12, s12, lat2, lon2)
            Console.WriteLine("{0} {1}", lat2, lon2)

            ' Sample inverse calculation, JFK to LHR
            lat1 = 40.6 : lon1 = -73.8 ' JFK Airport
            lat2 = 51.6 : lon2 = -0.5  ' LHR Airport
            rhumb.Inverse(lat1, lon1, lat2, lon2, s12, azi12)
            Console.WriteLine("{0} {1}", s12, azi12)
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
