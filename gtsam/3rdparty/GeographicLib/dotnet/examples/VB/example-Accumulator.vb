Imports NETGeographicLib

Module example_Accumulator
    Public Sub Main()
        Try
            ' Compare using Accumulator and ordinary summation for a sum of large and
            ' small terms.
            Dim sum As Double = 0.0
            Dim acc As Accumulator = New Accumulator()
            acc.Assign(0.0)
            sum += 1.0E+20 : sum += 1 : sum += 2 : sum += 100 : sum += 5000
            sum += -1.0E+20 : acc.Sum(1.0E+20) : acc.Sum(1) : acc.Sum(2) : acc.Sum(100) : acc.Sum(5000) : acc.Sum(-1.0E+20)
            Console.WriteLine(String.Format("{0} {1}", sum, acc.Result()))
        Catch ex As GeographicErr
            Console.WriteLine(String.Format("Caught exception: {0}", ex.Message))
        End Try
    End Sub
End Module
