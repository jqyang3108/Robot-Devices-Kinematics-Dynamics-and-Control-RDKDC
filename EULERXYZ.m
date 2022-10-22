function Ans = EULERXYZ(X)
% Input [Roll; Pitch; Yaw]
% Identifies Yaw with input z and outputs rotation matrix.
%

% Runs into an issue where there is a signularity when X(2) = -pi/2. This
% results in a matrix where if X(1)

Ans = ROTX(X(1))*ROTY(X(2))*ROTZ(X(3));