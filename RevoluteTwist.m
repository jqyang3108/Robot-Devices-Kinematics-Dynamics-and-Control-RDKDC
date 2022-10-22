function Ans = RevoluteTwist(q,w)
% Calculates Revolute twist from a given q and w.

X1 = cross(q,w);
X2 = w;

Ans = [X1;X2];