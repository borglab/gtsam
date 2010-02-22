function DOUBLES_EQUAL(expected,actual,atol)

if abs(expected-actual)>atol
  error(sprintf('DOUBLES_EQUAL fails: expected %g but got %g',expected,actual));
end
