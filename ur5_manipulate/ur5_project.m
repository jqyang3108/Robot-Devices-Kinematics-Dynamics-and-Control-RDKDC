%% input start and target positions here
gstart = [0 -1 0 0.47;
          0 0 1 0.55;
          -1 0 0 0.12;
          0 0 0 1];
gtarget = [0 -1 0 -0.30;
           0 0 1 0.39;
           -1 0 0 0.12;
           0 0 0 1];
% gstart = [0 -1 0 0.22;
%           0 0 1 0.3;
%           -1 0 0 0.40
%           0 0 0 1];
% gtarget = [0 -1 0 -0.2;
%            0 0 1 0.5;
%            -1 0 0 0.3;
%            0 0 0 1];
ur5 = ur5_interface();

%% move to the home configuration
home = [0.6; -2; 1; -1.5; -1;1];
ur5.move_joints(home,5);
display('Moved to home position. Press any key to start the task');
pause

%% move and place task
% choose the method for control trajectory: 1 = IK-based, 2 = DK-based, 3 = gradient-based
mode = 3;

% perform the task
if mode == 1
   [eRstart,erstart,eRtarget,ertarget] = ur5InvMovePlace(gstart,gtarget)
else
   [eRstart,erstart,eRtarget,ertarget] = ur5RRMovePlace(gstart,gtarget,ur5,mode)
end 