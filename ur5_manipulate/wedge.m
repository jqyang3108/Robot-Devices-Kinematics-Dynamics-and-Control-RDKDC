function Ans = wedge(X)
% Calculates the wedge of provided Twist(X)

A = SKEW3(X(4:6));
B = X(1:3);


Ans = [A B; 0 0 0 0];