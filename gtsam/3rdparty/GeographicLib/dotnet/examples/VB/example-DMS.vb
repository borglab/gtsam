Imports NETGeographicLib

Module example_DMS
    Sub Main()
        Try
            Dim desc As String = "30d14'45.6""S"
            Dim type As DMS.Flag
            Dim ang As Double = DMS.Decode(desc, type)
            Console.WriteLine(String.Format("Type: {0} String: {1}", type, ang))
            ang = -30.245715
            Dim prec As UInteger = 6
            desc = DMS.Encode(ang, prec, DMS.Flag.LATITUDE, 0)
            Console.WriteLine(String.Format("Latitude: {0}", desc))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
