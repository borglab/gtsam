function R = uth2rot(u,teta)

% Uso: R = uth2rot(u,teta)
%
% calcola la matrice R 
% a partire da asse u ed angolo di rotazione teta (in rad)

S=antisim(u);
t=teta;
 
n=norm(u);

R = eye(3) + sin(t)/n*S + (1-cos(t))/n^2*S^2;

