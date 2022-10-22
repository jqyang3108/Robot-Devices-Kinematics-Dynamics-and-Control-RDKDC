% input the solutions for inverse kinematics
% output the best joint angles to move to from the inverse kinematics solution 
function output = ur5InvKineSafetyCheck(solset)
[m,n] = size(solset);
output = [];
for i = 1:n
    theta1 = solset(1,i);
    theta2 = solset(2,i);
    theta3 = solset(3,i);
    theta4 = solset(4,i);
    theta5 = solset(5,i);
    %fprintf('\n\n----------Test solution %d----------\n\n',i)
%% Condition 1. Theta2 makes the robot hit surface;
    if(theta2>0&& theta2< -pi )
        %fprintf("----------\nAbort 1 theta %d = %.3f*pi: Robot hits surface\n----------",2,theta2/pi)
        continue;
%% Condition 2. link 2 coincides with robot
   elseif(abs(theta3) == pi)
        %fprintf("----------\nAbort 2 theta %d = %.3f*pi: Link1 coincides with Link2\n----------",3,theta3/pi)
        continue;
%% Condition 3. Theta 3 overrotates;
    elseif((theta3) >= 0)
        %fprintf("----------\nAbort 3 theta %d = %.3f*pi: Theta 3 overrotates\n----------",3,theta3/pi);
        continue;
%% Condition 4. Theta 1 overrotates;
    elseif((theta1) >=3*pi/2)  
        %fprintf("----------\nAbort 4 theta %d = %.3f*pi:Theta 1 overrotates\n----------",1,theta1/pi)
        continue;
%% Condition 5. Theta 4 overrotates;
    elseif((theta4) >= 0)    
        %fprintf("----------\nAbort 5 theta %d = %.3f*pi:Theta 4  overrotates\n----------",4,theta4/pi)
        continue;
%% Condition 6. Theta 5 overrotates;
    elseif((theta5) <=0 )   
        %fprintf("----------\nAbort 6 theta %d = %.3f*pi:Theta 5  overrotates\n----------",5,theta5/pi)
        continue;
%% Pass
    else
        %fprintf("----------\nSolution %d: works\n----------",i)
        output =[output i];
       
    end
end

if length(output) == 1
        output = output(1);
elseif length(output) > 1
        fprintf('\n\n Multiple possible solutions \n\n');
        return
else
        fprintf('\n\n No possible solutions \n\n');
        output = -1;
        return;
end
end