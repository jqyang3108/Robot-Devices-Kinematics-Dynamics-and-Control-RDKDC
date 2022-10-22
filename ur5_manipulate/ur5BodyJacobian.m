function Ans = ur5BodyJacobian(X)
% Input all 6 angles and outputs the resulting body Jacobian for the
% ur5 robot configuration


thet1 = X(1);
thet2 = X(2);
thet3 = X(3);
thet4 = X(4);
thet5 = X(5);
thet6 = X(6);

% Calculating ur5 configuration

% units are in mm
L0 = 0.0892;
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

w1 = [0;0;1];
q1 = [0;0;0];

w2 = [0;1;0];
q2 = [0;0;L0];

w3 = [0;1;0];
q3 = [L1;0;L0];

w4 = [0;1;0];
q4 = [L1+L2;L3;L0];

w5 = [0;0;-1];
q5 = [L1+L2;L3;L0-L4];

w6 = [0;1;0];
q6 = [L1+L2;L3+L5;L0-L4];

Twist{1} = RevoluteTwist(q1,w1);
Twist{2} = RevoluteTwist(q2,w2);
Twist{3} = RevoluteTwist(q3,w3);
Twist{4} = RevoluteTwist(q4,w4);
Twist{5} = RevoluteTwist(q5,w5);
Twist{6} = RevoluteTwist(q6,w6);


gst0 = [[0 1 0; 1 0 0; 0 0 -1] [ L1+L2; L3+L5;L0-L4]; 0 0 0 1];

Ans = [];
% This is done in reverse since this is body jacobian
for a = 1:6
    x = 6-(a-1);
    % Find inverse of gst
    R = gst0(1:3,1:3);
    p = gst0(1:3,4);
    ginv = inv(gst0);
    
    % Find Adjoint of inverse
    skewp = SKEW3(ginv(1:3,4));
    Rinv = ginv(1:3,1:3);
    Adginv = [Rinv skewp*Rinv; zeros(3) Rinv];
    
    % Compute column for jacobian by multiplying adjoint with twist and attatch
    Rcol = mtimes(Adginv,Twist{x});
    Ans = [Rcol Ans];
    
    %update gst for next one
    gst0 = expm(wedge(Twist{x})*X(x))*gst0;
end
inv(Ans);
    
    
    
    
    
    



