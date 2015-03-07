Imports NETGeographicLib

Module example_PolygonArea
    Sub Main()
        Try
            Dim geod As Geodesic = New Geodesic()  ' WGS84
            Dim poly As PolygonArea = New PolygonArea(geod, True)
            poly.AddPoint(52, 0)     ' London
            poly.AddPoint(41, -74)   ' New York
            poly.AddPoint(-23, -43)  ' Rio de Janeiro
            poly.AddPoint(-26, 28)   ' Johannesburg
            Dim perimeter, area As Double
            Dim n As UInteger = poly.Compute(False, True, perimeter, area)
            Console.WriteLine(String.Format("{0} {1} {2}", n, perimeter, area))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
