function key = symbol(c,i)
% generate a key corresponding to a symbol
s = gtsam.Symbol(c,i);
key = s.key();