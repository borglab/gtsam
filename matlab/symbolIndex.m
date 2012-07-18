function i = symbolIndex(key)
% generate the index from a key 
s = gtsam.Symbol(key);
i = s.index();