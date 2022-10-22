%% Check if provided location is ok to go towards
function Ans = safetycheck(X)
% Input 6 angles
% Return 1 if ok, Return 0 if not with error message.

% Check if not exceeding joint angles
Conditions(1) = (abs(X(1))<= pi());
Conditions(2) = (abs(X(2))<= pi());
Conditions(3) = (abs(X(3))<= 3*pi()/4); % putting this as 3pi()/4 so that the toolset doesn't overlap, can be more lenient here.
Conditions(4) = (abs(X(4))<= pi());
Conditions(5) = (abs(X(5))<= pi());
Conditions(6) = (abs(X(6))<= pi());
g = ur5FwdKin(X);
L1 = 0.425;
L2 = 0.392;
L4 = 0.09475;
% check if location is outside of limit
Conditions(7) = (norm(g(1:3,4))<=(L1+L2+L4));
% check if any location is too close to the robot
Conditions(8) = (norm(g(1:3,4))>= (0.1));
testvalue = all(Conditions);
Ans = 0;
if(testvalue == 1)
     % check if any location is under the surface
    if(g(3,4) >= 0)
        Ans = 1;
    end
end

if(Ans == 0)
    error('Your goal value cannot be reached with the current setup. Please input a different value.');
end


