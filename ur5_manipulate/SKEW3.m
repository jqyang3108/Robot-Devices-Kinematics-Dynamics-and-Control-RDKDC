function Ans = SKEW3(X)
% Input a 3x1 vector and returns canonical 3x3 skew-symmetric matrix given
% in MLS and also in class


Ans = [ 0 -X(3) X(2); X(3) 0 -X(1); -X(2) X(1) 0];