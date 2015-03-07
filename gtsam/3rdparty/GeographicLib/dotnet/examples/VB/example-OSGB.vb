Imports NETGeographicLib

Module example_OSGB
    Sub Main()
        Try
            ' Sample forward calculation from
            ' A guide to coordinate systems in Great Britain
            Dim lat As Double = DMS.Decode(52, 39, 27.2531)
            Dim lon As Double = DMS.Decode(1, 43, 4.5177)
            Dim x, y As Double
            OSGB.Forward(lat, lon, x, y)
            Dim gridref As String = ""
            OSGB.GridReference(x, y, 2, gridref)
            Console.WriteLine(String.Format("{0} {1} {2}", x, y, gridref))
            ' Sample reverse calculation
            gridref = "TG5113"
            Dim prec As Integer
            OSGB.GridReference(gridref, x, y, prec, True)
            OSGB.Reverse(x, y, lat, lon)
            Console.WriteLine(String.Format("{0} {1} {2}", prec, lat, lon))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
