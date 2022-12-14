function [eRstart,erstart,eRtarget,ertarget] = ur5InvMovePlace(gstart,gtarget)

rosshutdown;
ur5 = ur5_interface();

%% Parameters & Frames
time = 8;
Frame_start = tf_frame('base_link','start',gstart);
pause(0.5)

Frame_base = tf_frame('base_link','target',gtarget);
pause(0.5)


%% Calculate solutions
g_T_toolK = [ROTY(pi/2)*ROTZ(-pi/2) [0 0 0]'; 0 0 0 1]; %transformation from {T} to keating tool 

%start = ur5InvKin(gstart);
start = ur5InvKin(gstart*g_T_toolK);
gstartcopy = gstart; 
gstart(3,4) = gstart(3,4) + 0.2;
ggrab = gstart;
grab = ur5InvKin(ggrab*g_T_toolK);

%target = ur5InvKin(gtarget);
target = ur5InvKin(gtarget*g_T_toolK);
gtargetcopy = gtarget
gtarget(3,4)= gtarget(3,4) + 0.2;
grelease =gtarget;
release = ur5InvKin(grelease*g_T_toolK);

%% Safety check
L1 = 0.425;
L2 = 0.392;
L4 = 0.09475;

% check if any location is outside the range of robot
if (norm(gstart(1:3,4))> (L1+L2+L4))||(norm(ggrab(1:3,4))>  (L1+L2+L4))||(norm(gtarget(1:3,4))>  (L1+L2+L4))||(norm(grelease(1:3,4))>  (L1+L2+L4))
   fprintf('\n\n One location is unreachable to the robot! \n\n');
   return
% check if any location is too close to the robot
elseif (norm(gstart(1:3,4))< (0.1))||(norm(ggrab(1:3,4))< (0.1))||(norm(gtarget(1:3,4)) < (0.1))||(norm(grelease(1:3,4))< (0.1))
   fprintf('\n\n One location is too close to the robot! \n\n');
    return
 % check if any location is under the surface
elseif(gstart(3,4)<0.001)||(ggrab(3,4)<0.001)||(gtarget(3,4)<0.001)||(grelease(3,4)<0.001)
    fprintf('\n\n One location is under the surface! \n\n');
    return
else
    fprintf('\n\n Locations passed safety check')
end


%% Step 1: Move above the start location.
fwdKinToolFrame = tf_frame("base_link","AboveStart",ggrab);
fprintf('\n Step 1: Move above the start location. Press any key to continue \n');
ur5.move_joints(grab(:,ur5InvKineSafetyCheck(grab)),time)
pause

%% Step 2: Move straight down to the start location.
fprintf('\n Step 2: Move straight down to the start location.  Press any key to continue\n');
ur5.move_joints(start(:,ur5InvKineSafetyCheck(start)),time)
pause
% calculate error for start pose
X = gstartcopy;
Rd = X(1:3,1:3); rd = X(1:3,4);
gcur = ur5FwdKin(ur5.get_current_joints());
R = gcur(1:3,1:3); r = gcur(1:3,4);
eRstart = sqrt(trace((R-Rd)*transpose(R-Rd)));
erstart = norm(r-rd);

%% Step 3: Move back to the previous pose.
fprintf('\n Step 3: Move back to the previous pose.\n');
ur5.move_joints(grab(:,ur5InvKineSafetyCheck(grab)),time)
pause

%% Step 4: Move above the target location
fprintf('\n Step 4: Move above the target location. Press any key to continue\n');
abovegoaltarget = tf_frame("base_link","AboveTarget",gtarget);
ur5.move_joints(release(:,ur5InvKineSafetyCheck(release)),time)
pause

%% Step 5: Move straight down to the target location.
fprintf('\n Step 5: Move straight down to the start location. Press any key to continue \n');
ur5.move_joints(target(:,ur5InvKineSafetyCheck(target)),time)
pause
% calculate error for target pose
Y = gtargetcopy;
Rd = Y(1:3,1:3); rd = Y(1:3,4);
gcur = ur5FwdKin(ur5.get_current_joints());
R = gcur(1:3,1:3); r = gcur(1:3,4);
eRtarget = sqrt(trace((R-Rd)*transpose(R-Rd)));
ertarget = norm(r-rd);

%% Step 6: Move back to the previous pose.
fprintf('\n Step 6: Move back to the previous pose. \n');
ur5.move_joints(release(:,ur5InvKineSafetyCheck(release)),time)
fprintf('\n Task complete \n');

end