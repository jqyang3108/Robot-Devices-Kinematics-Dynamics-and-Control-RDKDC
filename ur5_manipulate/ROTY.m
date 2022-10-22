function Ans = ROTY(y)
%
% Identifies Pitch with input y and outputs rotation matrix.
%

Ans = [ cos(y) 0 sin(y); 0 1 0; -sin(y) 0 cos(y)];