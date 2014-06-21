function S = antisim(v)

% costruisce la matrice antisimmetrica S (3x3) a partire dal vettore v
% Uso: S = antisim(v)

S=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];

