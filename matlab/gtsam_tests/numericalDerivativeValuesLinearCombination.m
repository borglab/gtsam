function y = numericalDerivativeValuesLinearCombination(values, key_a, key_b)
% Test helper for a linear combination of two Values entries.
a = values.atVector(key_a);
b = values.atVector(key_b);
y = [a(1) + 2.0 * a(2) + 3.0 * b;
     -4.0 * a(1) + 5.0 * b];
end
