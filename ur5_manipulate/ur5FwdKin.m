function Ans = ur5FwdKin(X)
% Input all 6 angles and outputs the resulting forward kinematics for the
% ur5 robot configuration


thet1 = X(1);
thet2 = X(2);
thet3 = X(3);
thet4 = X(4);
thet5 = X(5);
thet6 = X(6);

% Calculating ur5 configuration
% start by listing parameters, these are listed in meters, not in mm,
% Referenced from Hw6 chart

% The twists are identified through visual inspection.

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

% Calculating twists using revolute twist, there are no prismatic twists.
Twist1 = RevoluteTwist(q1,w1);
Twist2 = RevoluteTwist(q2,w2);
Twist3 = RevoluteTwist(q3,w3);
Twist4 = RevoluteTwist(q4,w4);
Twist5 = RevoluteTwist(q5,w5);
Twist6 = RevoluteTwist(q6,w6);

% Using gst0 through visual inspection, no rotation of frame, and the
% toolframe is located at X:L3+L5, and 
gst0 = [[0 1 0; 1 0 0; 0 0 -1] [ L1+L2; L3+L5;L0-L4]; 0 0 0 1];


% Using forward Kinematics Equation through expm() (exponential matrix) and
% wedge, which converts Twists into se(3), multiplying them. This equation
% is given in MLS.

Ans = expm(wedge(Twist1)*thet1)*expm(wedge(Twist2)*thet2)*expm(wedge(Twist3)*thet3)...
    *expm(wedge(Twist4)*thet4)*expm(wedge(Twist5)*thet5)*expm(wedge(Twist6)*thet6)*gst0;



