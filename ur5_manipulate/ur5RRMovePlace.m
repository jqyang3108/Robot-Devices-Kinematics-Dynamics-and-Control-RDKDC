%% Final Project ResolvedRate Control Inverse Jacobian
function [eRstart,erstart,eRtarget,ertarget] = ur5RRMovePlace(X,Y,ur5,mode)
% Input X: Homogeneous start location
% Input Y: Homogeneous target location
% OutputAns: Returns 1 if succesful, returns 0 if not.

% Checking if input is good or not

%% setting up the parameters
dist = 0.2;
if mode == 2
    K = 20;
elseif mode == 3
    K = 50;
end
gdesired = X + [0 0 0 0; 0 0 0 0; 0 0 0 dist; 0 0 0 0];
gtarget = Y + [0 0 0 0; 0 0 0 0; 0 0 0 dist; 0 0 0 0];
start = tf_frame("base_link","start",X);
pause(0.5)
target = tf_frame("base_link","target",Y);
pause(0.5)
%% Moving above start location
fwdKinToolFrame = tf_frame("base_link","AboveStart",gdesired);
ur5RRcontrol(gdesired, K, ur5 ,mode)
display('Moved above start location, press any key to continue');
pause;

%% Moving down to start location
ur5RRcontrol(X, K, ur5 ,mode)

% calculate error for start pose
Rd = X(1:3,1:3); rd = X(1:3,4);
pause;
gcur = ur5FwdKin(ur5.get_current_joints());
R = gcur(1:3,1:3); r = gcur(1:3,4);
eRstart = sqrt(trace((R-Rd)*transpose(R-Rd)));
erstart = norm(r-rd);
display('Moved to start location, press any key to continue');


%% Moving above start location
ur5RRcontrol(gdesired, K, ur5 ,mode)
display('Moved above start location, press any key to continue');
pause;

%% Moving over to above target location
abovegoaltarget = tf_frame("base_link","AboveTarget",gtarget);
ur5RRcontrol(gtarget, K, ur5 ,mode)
display('Moved above target location, press any key to continue');
pause;

%% Moving to target location
ur5RRcontrol(Y, K, ur5 ,mode)

% calculate error for target pose
Rd = Y(1:3,1:3); rd = Y(1:3,4);
pause;
gcur = ur5FwdKin(ur5.get_current_joints());
R = gcur(1:3,1:3); r = gcur(1:3,4);
eRtarget = sqrt(trace((R-Rd)*transpose(R-Rd)));
ertarget = norm(r-rd);
display('Moved to target location, press any key to continue');

%% Moving over to above target location
ur5RRcontrol(gtarget, K, ur5, mode)
display('Moved above target location, task completed');

end
