function Ans = ROTZ(z)
%
% Identifies Yaw with input z and outputs rotation matrix.
%

Ans = [ cos(z) -sin(z) 0; sin(z) cos(z) 0; 0 0 1];