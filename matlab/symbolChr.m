function c = symbolChr(key)
% generate the chr from a key 
s = gtsam.Symbol(key);
c = s.chr();