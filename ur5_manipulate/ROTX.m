function Ans = ROTX(x)
%
% Identifies Roll with input X and outputs rotation matrix.
%

Ans = [ 1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
